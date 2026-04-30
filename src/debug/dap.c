#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <inttypes.h>

#include "../assert.h"
#include "../device/cpu/general_cpu.h"
#include "../fault.h"
#include "../main.h"
#include "dap.h"

// be64toh is not on macOS
#if defined(__APPLE__)
  #include <libkern/OSByteOrder.h>
  #define be64toh(x) OSSwapBigToHostInt64(x)
  #define htobe64(x) OSSwapHostToBigInt64(x)
#endif

static int connection_fd = -1;
static uint32_t cpuno_global = 0; // Default CPU device number used
static uint64_t steps_left = 0; // Number of instruction steps to execute before pausing. 0 means infinite.

typedef enum dap_request_type {
    /** Request to resume execution. Also used for the initial start. */
    ResumeRequest = 0x01,
    /** Request to pause execution. */
    PauseRequest = 0x02,
    /** Request to stop execution and exit the simulator. */
    StopRequest = 0x03,
    /** Request to step `arg0=count` instructions. */
    StepRequest = 0x04,

    // Breakpoint requests
    /** Request to set a code breakpoint at `arg0=address`. */
    SetCodeBreakpointRequest = 0x05,
    /** Request to remove a code breakpoint at `arg0=address`. */
    RemoveCodeBreakpointRequest = 0x06,

    /** Request to set a data breakpoint at `arg0=address` with `arg1=kind`. */
    SetDataBreakpointRequest = 0x07,
    /** Request to remove a data breakpoint at `arg0=address`. */
    RemoveDataBreakpointRequest = 0x08,

    // Register requests
    /** Request to read the value of register `arg0=id`. */
    ReadGeneralRegisterRequest = 0x09,
    /** Request to write to register `arg0=id` the value `arg1=value`. */
    WriteGeneralRegisterRequest = 0x0A,

    /** Request to read the value of Control and Status Register (CSR) `arg0=id`. */
    ReadCsrRequest = 0x0B,
    /** Request to write to Control and Status Register (CSR) `arg0=id` the value `arg1=value`. */
    WriteCsrRequest = 0x0C,

    /** Request to read the value of the program counter. */
    ReadPCRequest = 0x0D,
    /** Request to write `arg0=value` to the program counter. */
    WritePCRequest = 0x0E,

    // Memory requests
    /** Request to read physical memory at `arg0=address`. */
    ReadPhysMemoryRequest = 0x0F,
    /** Request to write `arg1=data` (8 B) to physical memory at `arg0=address`. */
    WritePhysMemoryRequest = 0x10,

    /** Request to read virtual memory at `arg0=address`. */
    ReadVirtMemoryRequest = 0x11,
    /** Request to write `arg1=data` (8 B) to virtual memory at `arg0=address`. */
    WriteVirtMemoryRequest = 0x12,

    /** Request to translate the virtual address `arg0=address` to a physical address. */
    TranslateAddressRequest = 0x13,

    // Interrupt requests
    /** Request to raise interrupt `arg0=id`. */
    RaiseInterruptRequest = 0x14,
    /** Request to clear interrupt `arg0=id`. */
    ClearInterruptRequest = 0x15,

    /** Request to get the value of configuration */
    GetConfigRequest = 0x16,
} dap_request_type_t;

typedef enum dap_outbound_category {
    /** Response to a request frame. */
    ResponseCategory = 0x01,
    /** Event frame. */
    EventCategory = 0x02
} dap_outbound_category_t;

typedef enum dap_response_status {
    /** Response to a successful request. */
    StatusOk = 0x01,
    /** Response to a failed request with an unspecified error. */
    StatusUnspecifiedError = 0x02,
    /** Response to an unsupported request. */
    StatusUnsupportedRequestError = 0x03
} dap_response_status_t;

typedef enum dap_event_type {
    /** Event indicating that the simulator has exited. */
    ExitedEvent = 0x01,
    /** Event indicating that the simulator has paused execution. */
    StoppedAtEvent = 0x02,
} dap_event_type_t;

typedef enum dap_stopped_reason {
    /** Stopped due to a pause. */
    StoppedReasonPaused = 0x01,
    /** Stopped due to hitting a breakpoint. */
    StoppedReasonBreakpoint = 0x02,
    /** Stopped due to stepping. */
    StoppedReasonStep = 0x03,
    /** Stopped due to an interrupt. */
    StoppedReasonInterrupt = 0x04,

} dap_stopped_reason_t;

/** Structure for DAP requests */
typedef struct dap_request {
    dap_request_type_t type;
    uint64_t arg0;
    uint64_t arg1;
    uint64_t arg2;
} dap_request_t;

/** Structure for DAP responses */
typedef struct dap_response {
    dap_response_status_t type;
    uint64_t arg0;
    uint64_t arg1;
    uint64_t arg2;
} dap_response_t;

/** Structure for DAP events */
typedef struct dap_event {
    dap_event_type_t type;
    uint64_t arg0;
    uint64_t arg1;
    uint64_t arg2;
} dap_event_t;

enum {
    INBOUND_FRAME_SIZE = 25, /** Size of a single DAP request frame */
    OUTBOUND_FRAME_SIZE = 26, /** Size of a single DAP response frame */
};

/** Length of an inbound frame is always 25 B = 1 B (type) + 8 B (arg0) + 8 B (arg1) + 8 B (arg2) */
static_assert(INBOUND_FRAME_SIZE == sizeof(uint8_t) + 3 * sizeof(uint64_t), "DAP inbound frame must be exactly 25 bytes long");

/** Length of an outbound frame is always 26 B = 1 B (category) + 1 B (status/type) + 8 B (arg0) + 8 B (arg1) + 8 B (arg2) */
static_assert(OUTBOUND_FRAME_SIZE == sizeof(uint8_t) + sizeof(uint8_t) + 3 * sizeof(uint64_t), "DAP inbound frame must be exactly 26 bytes long");

/** Internal buffer for incoming frames */
static uint8_t frame_buffer[INBOUND_FRAME_SIZE];
static ssize_t frame_buffered_bytes = 0; // Number of bytes currently buffered in frame buffer

bool dap_init(void)
{
    const int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        io_error("dap_socket");
        return false;
    }

    const int yes = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes))) {
        io_error("setsockopt");
        return false;
    }

    struct sockaddr_in address = { 0 };
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(dap_port);

    if (bind(sock, (struct sockaddr *) &address, sizeof(address)) < 0) {
        io_error("dap_bind");
        return false;
    }

    if (listen(sock, 1) < 0) {
        io_error("dap_listen");
        return false;
    }

    alert("Listening for DAP connection on port %u.", dap_port);

    struct sockaddr_in sa_dap;
    socklen_t address_len = sizeof(sa_dap);
    connection_fd = accept(sock, (struct sockaddr *) &sa_dap, &address_len);
    if (connection_fd < 0) {
        if (errno == EINTR) {
            alert("DAP: Interrupted");
        } else {
            io_error("dap_accept");
        }

        return false;
    }

    steps_left = 0;

    alert("DAP connected.");
    return true;
}

/** Receive bytes from DAP connection
 *
 * This will try to receive `INBOUND_FRAME_SIZE` bytes and store
 * them in `buf`.
 * If not enough bytes are available, it will return false and
 * buffer the received bytes internally until enough bytes are
 * available. The buffer provided is not modified in this case.
 * @param buf Pointer to buffer for received data. Only modified if the function returns true.
 * @param block Whether to block until bytes are available.
 * If false, the function will return immediately if no bytes are available, without buffering.
 * @return True if successfully received `INBOUND_FRAME_SIZE` bytes and stored them in `buf`.
 * False otherwise.
 */
static bool dap_receive_bytes(void* buf, const bool block)
{
    const ssize_t need_bytes = INBOUND_FRAME_SIZE - frame_buffered_bytes;
    uint8_t* write_buffer = frame_buffer + frame_buffered_bytes;
    ASSERT(need_bytes <= INBOUND_FRAME_SIZE);
    ASSERT(0 < need_bytes);
    ASSERT(connection_fd != -1);
    ASSERT(buf != NULL);
    ASSERT(write_buffer + need_bytes <= frame_buffer + INBOUND_FRAME_SIZE); // No overflow check

    const int flags = block ? 0 : MSG_DONTWAIT;
    const ssize_t received = recv(connection_fd, write_buffer, need_bytes, flags);

    // Received
    if (received > 0) {
        frame_buffered_bytes += received;

        // Got enough
        if (received == need_bytes) {
            ASSERT(frame_buffered_bytes == INBOUND_FRAME_SIZE);
            memcpy(buf, frame_buffer, INBOUND_FRAME_SIZE);
            frame_buffered_bytes = 0;
            return true;
        }

        // Not enough, buffer
        ASSERT(received < need_bytes);
        ASSERT(frame_buffered_bytes < INBOUND_FRAME_SIZE);
        return false;
    }

    // No bytes for now
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return false;
    }

    // Closed or had errors, exit

    if (received < 0) {
        io_error("dap_recv");
    }
    dap_state = DAP_DONE;
    return false;
}

/** Send bytes to DAP connection, blocking until all successfully sent
 *
 * This will try to send `OUTBOUND_FRAME_SIZE` bytes from `buf`.
 * If not all bytes can be sent at once, it will block and retry
 * until all bytes are sent.
 * The buffer provided is not modified until the operation is
 * successful.
 * @param buf Pointer to buffer with data to send.
 * @return True if successful.
 */
static bool dap_send_bytes(const void* buf)
{
    ssize_t to_send = OUTBOUND_FRAME_SIZE;
    const uint8_t* ptr = buf;
    while (to_send > 0) {
        const ssize_t sent = send(connection_fd, ptr, to_send, 0);
        // Error
        if (sent < 0) {
            if (errno == EINTR) continue;
            io_error("dap_send");
            dap_state = DAP_DONE;
            return false;
        }
        // Connection closed
        if (sent == 0) {
            dap_state = DAP_DONE;
            return false;
        }
        ptr += sent;
        to_send -= sent;
    }
    return true;
}

/** Receive a DAP request
 *
 * @param out_cmd Pointer to output request structure. Only modified if the function returns true.
 * @param block Whether to block until a full request is available.
 * @return True if successfully received a full request and stored it in `out_cmd`. False otherwise.
 */
static bool dap_receive_request(dap_request_t *out_cmd, const bool block)
{
    uint8_t buffer[INBOUND_FRAME_SIZE] = { 0 };
    if (!dap_receive_bytes(buffer, block)) {
        return false;
    }

    out_cmd->type = buffer[0];

    uint64_t arg0_no;
    uint64_t arg1_no;
    uint64_t arg2_no;
    memcpy(&arg0_no, buffer + sizeof(uint8_t), sizeof(arg0_no));
    memcpy(&arg1_no, buffer + sizeof(uint8_t) + sizeof(arg0_no), sizeof(arg1_no));
    memcpy(&arg2_no, buffer + sizeof(uint8_t) + sizeof(arg0_no) + sizeof(arg1_no), sizeof(arg2_no));

    out_cmd->arg0 = be64toh(arg0_no);
    out_cmd->arg1 = be64toh(arg1_no);
    out_cmd->arg2 = be64toh(arg2_no);

    return true;
}

/** Send a DAP response, blocking until successful
 *
 * @param response Response to send.
 * @return True if successful.
 */
static bool dap_send_response(const dap_response_t response)
{
    uint8_t buffer[OUTBOUND_FRAME_SIZE] = { 0 };
    buffer[0] = ResponseCategory;
    buffer[1] = (uint8_t)response.type;

    const uint64_t arg0_no = htobe64(response.arg0);
    const uint64_t arg1_no = htobe64(response.arg1);
    const uint64_t arg2_no = htobe64(response.arg2);
    memcpy(buffer + 2, &arg0_no, sizeof(arg0_no));
    memcpy(buffer + 2 + sizeof(arg0_no), &arg1_no, sizeof(arg1_no));
    memcpy(buffer + 2 + sizeof(arg0_no) + sizeof(arg1_no), &arg2_no, sizeof(arg2_no));

    return dap_send_bytes(buffer);
}

/** Send a DAP event, blocking until successful
 *
 * @param event Event to send.
 * @return True if successful.
 */
static bool dap_send_event(const dap_event_t event)
{
    uint8_t buffer[OUTBOUND_FRAME_SIZE] = { 0 };
    buffer[0] = EventCategory;
    buffer[1] = (uint8_t)event.type;

    const uint64_t arg0_no = htobe64(event.arg0);
    const uint64_t arg1_no = htobe64(event.arg1);
    const uint64_t arg2_no = htobe64(event.arg2);
    memcpy(buffer + 2, &arg0_no, sizeof(arg0_no));
    memcpy(buffer + 2 + sizeof(arg0_no), &arg1_no, sizeof(arg1_no));
    memcpy(buffer + 2 + sizeof(arg0_no) + sizeof(arg1_no), &arg2_no, sizeof(arg2_no));

    return dap_send_bytes(buffer);
}

void dap_close(void)
{
    if (connection_fd != -1) {
        dap_send_event((dap_event_t){ExitedEvent, 0x00, 0x00, 0x00});

        if (close(connection_fd) == -1) {
            io_error("dap_connection_fd");
        }
        connection_fd = -1;
    }

    dap_state = DAP_DONE;
    machine_halt = true;
    alert("DAP connection closed.");
}

/* Simulator events */

void dap_event_hit_code_breakpoint(const uint64_t address)
{
    dap_state = DAP_PAUSED;
    dap_send_event((dap_event_t){StoppedAtEvent, address, StoppedReasonBreakpoint, 0x00});
}

/* Handlers */

static void dap_handle_resume(void)
{
    dap_state = DAP_RUNNING;
    alert("DAP: Resuming execution.");
    dap_send_response((dap_response_t){ StatusOk, 0x00, 0x00, 0x00});
}

static void dap_handle_pause(void)
{
    dap_state = DAP_PAUSED;
    alert("DAP: Pausing execution.");
    dap_send_response((dap_response_t){ StatusOk, 0x00, 0x00, 0x00});
    const ptr64_t address = cpu_get_pc(get_cpu(cpuno_global));
    dap_send_event((dap_event_t){StoppedAtEvent, address.ptr, StoppedReasonPaused, 0x00});
}

/** Handle set code breakpoint request */
static void dap_handle_set_code_breakpoint(const uint64_t addr)
{
    const ptr64_t virt_addr = { addr };
    general_cpu_t* cpu = get_cpu(cpuno_global);

    if (!cpu_insert_breakpoint(cpu, virt_addr, BREAKPOINT_KIND_DEBUGGER)) {
        dap_send_response((dap_response_t){ StatusUnspecifiedError, 0x00, 0x00, 0x00}); // TODO: more specific err code
        return;
    }

    alert("Added DAP code breakpoint at address %#0" PRIx64, virt_addr.ptr);
    dap_send_response((dap_response_t){ StatusOk, 0x00, 0x00, 0x00});
}

/** Handle remove code breakpoint request */
static void dap_handle_remove_code_breakpoint(const uint64_t addr)
{
    const ptr64_t virt_addr = { addr };
    general_cpu_t* cpu = get_cpu(cpuno_global);

    if (!cpu_remove_breakpoint(cpu, virt_addr, BREAKPOINT_KIND_DEBUGGER)) {
        alert("No such breakpoint!");
        dap_send_response((dap_response_t){ StatusUnspecifiedError, 0x00, 0x00, 0x00}); // TODO: more specific err code
        return;
    }

    alert("Removed DAP code breakpoint from address %#0" PRIx64, virt_addr.ptr);
    dap_send_response((dap_response_t){ StatusOk, 0x00, 0x00 , 0x00});
}

static void dap_handle_step(const uint64_t count)
{
    steps_left = count;
    dap_state = DAP_RUNNING;
    alert("DAP: Stepping %lu instructions.", count);
    dap_send_response((dap_response_t){ StatusOk, 0x00, 0x00, 0x00});
}

static void dap_handle_get_config(void)
{
    uint64_t cpu_count = 0;
    general_cpu_t *cpu;
    for_each(cpu_list, cpu, general_cpu_t) {
        ++cpu_count;
    }

    dap_send_response((dap_response_t){ StatusOk, cpu_count, 0x00, 0x00});
}

static void dap_check_step(void)
{
    if (steps_left > 0) {
        --steps_left;

        if (steps_left == 0) {
            dap_state = DAP_PAUSED;
            const ptr64_t address = cpu_get_pc(get_cpu(cpuno_global));
            dap_send_event((dap_event_t){StoppedAtEvent, address.ptr, StoppedReasonStep, 0x00});
        }
    }
}

void dap_process(void)
{
    dap_check_step();

    dap_request_t request = { 0 };

    while (dap_receive_request(&request, dap_state == DAP_PAUSED)) {
        switch (request.type) {
        case ResumeRequest:
            dap_handle_resume();
            continue;
        case PauseRequest:
            dap_handle_pause();
            continue;
        case StopRequest:
            // Response is handled in dap_close which is always called at the end.
            dap_state = DAP_DONE;
            return;
        case StepRequest:
            dap_handle_step(request.arg0);
            continue;
        case SetCodeBreakpointRequest:
            dap_handle_set_code_breakpoint(request.arg0);
            continue;
        case RemoveCodeBreakpointRequest:
            dap_handle_remove_code_breakpoint(request.arg0);
            continue;
        case GetConfigRequest:
            dap_handle_get_config();
            continue;
        default:
            alert("Unknown DAP request type %u.", request.type);
            dap_send_response((dap_response_t){ StatusUnsupportedRequestError, 0x00, 0x00, 0x00});
        }
    }
}
