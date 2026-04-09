#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "../assert.h"
#include "../device/cpu/general_cpu.h"
#include "../device/cpu/mips_r4000/cpu.h"
#include "../device/cpu/riscv_rv32ima/cpu.h"
#include "../fault.h"
#include "../main.h"
#include "dap.h"

static int connection_fd = -1;
static unsigned int cpuno_global = 0; // TODO: what is it?

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

typedef enum dap_event_kind {
    /** Event indicating that the simulator has exited. */
    ExitedEvent = 0x01,
    /** Event indicating that the simulator has paused execution. */
    StoppedAtEvent = 0x02,
} dap_event_type_t;

/** Structure for DAP requests */
typedef struct dap_request {
    dap_request_type_t type;
    uint64_t arg0;
    uint64_t arg1;
} dap_request_t;

/** Structure for DAP responses */
typedef struct dap_response {
    dap_response_status_t type;
    uint64_t arg0;
    uint64_t arg1;
} dap_response_t;

/** Structure for DAP events */
typedef struct dap_event {
    dap_event_type_t type;
    uint64_t arg0;
    uint64_t arg1;
} dap_event_t;

enum {
    INBOUND_FRAME_SIZE = 17, /** Size of a single DAP request frame */
    OUTBOUND_FRAME_SIZE = 18, /** Size of a single DAP response frame */
};

/** Length of an inbound frame is always 17 B = 1 B (type) + 8 B (arg0) + 8 B (arg1) */
static_assert(INBOUND_FRAME_SIZE == sizeof(uint8_t) + 2 * sizeof(uint64_t), "DAP inbound frame must be exactly 17 bytes long");

/** Length of an outbound frame is always 17 B = 1 B (category) + 1 B (status/type) + 8 B (arg0) + 8 B (arg1) */
static_assert(OUTBOUND_FRAME_SIZE == sizeof(uint8_t) + sizeof(uint8_t) + 2 * sizeof(uint64_t), "DAP inbound frame must be exactly 18 bytes long");

/** Internal buffer for incoming frames */
static uint8_t frame_buffer[INBOUND_FRAME_SIZE];
static const ssize_t frame_buffer_len = INBOUND_FRAME_SIZE; // TODO: useless
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

    alert("DAP connected.");
    return true;
}

void dap_close(void)
{
    if (connection_fd == -1) {
        io_error("dap_already_closed");
    }

    if (close(connection_fd) == -1) {
        io_error("dap_connection_fd");
    }

    connection_fd = -1;
    dap_state = DAP_DONE;
    machine_halt = true;
    alert("DAP connection closed.");
}

/** Receive bytes from DAP connection, non-blocking
 *
 * This will try to receive `len` bytes and store them in `buf`.
 * If not enough bytes are available, it will return false and
 * buffer the received bytes internally until enough bytes are
 * available. The buffer provided is not touched in this case.
 * @param buf Pointer to buffer for received data.
 * @param len Total number of bytes to receive.
 * @return True if successful.
 */
// TODO: maybe just simplify to buffer 17B as the protocol is fixed sized
static bool dap_receive_bytes(void* buf, const ssize_t len)
{
    const ssize_t need_bytes = len - frame_buffered_bytes;
    uint8_t* write_buffer = frame_buffer + frame_buffered_bytes;
    ASSERT(need_bytes <= frame_buffer_len);
    ASSERT(len <= frame_buffer_len);
    ASSERT(need_bytes > 0);
    ASSERT(connection_fd != -1);
    ASSERT(buf != NULL);
    ASSERT(len > 0);
    ASSERT(write_buffer + need_bytes <= frame_buffer + frame_buffer_len); // No overflow check

    const ssize_t received = recv(connection_fd, write_buffer, need_bytes, MSG_DONTWAIT);

    // Received
    if (received > 0) {
        frame_buffered_bytes += received;

        // Got enough
        if (received == need_bytes) {
            ASSERT(frame_buffered_bytes == len);
            memcpy(buf, frame_buffer, len);
            frame_buffered_bytes = 0;
            return true;
        }

        // Not enough, buffer
        ASSERT(received < need_bytes);
        ASSERT(frame_buffered_bytes < len);
        ASSERT(frame_buffered_bytes < frame_buffer_len);
        return false;
    }

    // Closed
    if (received == 0) {
        dap_close();
        return false;
    }

    // No bytes for now
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;
    }

    // Error
    io_error("dap_recv");
    return false;
}

/** Send bytes to DAP connection, blocking until all successfully sent
 *
 * This will try to send `len` bytes from `buf`. If not all bytes
 * can be sent at once, it will block and retry until all bytes
 * are sent. The buffer provided is not modified until the operation is
 * successful.
 * @param buf Pointer to buffer with data to send.
 * @param len Total number of bytes to send.
 * @return True if successful.
 */
static bool dap_send_bytes(const void* buf, ssize_t len)
{
    const uint8_t* ptr = buf;
    while (len > 0) {
        const ssize_t sent = send(connection_fd, ptr, len, 0);
        if (sent < 0) {
            if (errno == EINTR) continue;
            io_error("dap_send");
            return false;
        }
        if (sent == 0) {
            dap_close();
            return false;
        }
        ptr += sent;
        len -= sent;
    }
    return true;
}

/** Receive a DAP request, non-blocking
 *
 * @param out_cmd Pointer to output request structure.
 * @return True if successful.
 */
static bool dap_receive_request(dap_request_t *out_cmd)
{
    uint8_t buffer[sizeof(dap_request_t)] = { 0 };
    if (!dap_receive_bytes(buffer, INBOUND_FRAME_SIZE)) {
        return false;
    }

    out_cmd->type = buffer[0];

    uint64_t netorder_arg0;
    uint64_t netorder_arg1;
    memcpy(&netorder_arg0, buffer + sizeof(out_cmd->type), sizeof(netorder_arg0));
    memcpy(&netorder_arg1, buffer + sizeof(out_cmd->type) + sizeof(out_cmd->arg0), sizeof(netorder_arg1));

    out_cmd->arg0 = be64toh(netorder_arg0);
    out_cmd->arg1 = be64toh(netorder_arg1);

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

    const uint64_t netorder_arg0 = htobe64(response.arg0);
    const uint64_t netorder_arg1 = htobe64(response.arg1);
    memcpy(buffer + 2, &netorder_arg0, sizeof(netorder_arg0));
    memcpy(buffer + 2 + sizeof(netorder_arg0), &netorder_arg1, sizeof(netorder_arg1));

    return dap_send_bytes(buffer, sizeof(buffer));
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

    const uint64_t netorder_arg0 = htobe64(event.arg0);
    const uint64_t netorder_arg1 = htobe64(event.arg1);
    memcpy(buffer + 2, &netorder_arg0, sizeof(netorder_arg0));
    memcpy(buffer + 2 + sizeof(netorder_arg0), &netorder_arg1, sizeof(netorder_arg1));

    return dap_send_bytes(buffer, sizeof(buffer));
}

static void dap_handle_resume(void)
{
    dap_state = DAP_RUNNING;
    alert("DAP: Resuming execution.");
    dap_send_response((dap_response_t){ .type = StatusOk, .arg0 = 0, .arg1 = 0 });
}

// TODO: implement both set and remove as device-specific (vtable) entries and use that to support all archs
/** Set a DAP code breakpoint */
static void dap_handle_set_code_breakpoint(const uint64_t addr)
{
    ptr64_t virt_address = { addr };
    virt_address.hi = 0; // TODO: HACK for 32bit, support 64-bit addresses
    rv_cpu_t *cpu = get_cpu(cpuno_global)->data;

    // TODO: Register as DAP debugger breakpoint
    const breakpoint_t *breakpoint = breakpoint_find_by_address(cpu->bps, virt_address, BREAKPOINT_FILTER_SIMULATOR);
    if (breakpoint != NULL) {
        dap_send_response((dap_response_t){ StatusUnspecifiedError, 0, 0 }); // TODO: more specific err code
        return;
    }

    breakpoint_t *inserted_breakpoint = breakpoint_init(virt_address, BREAKPOINT_KIND_SIMULATOR);
    list_append(&cpu->bps, &inserted_breakpoint->item);
    alert("Added DAP code breakpoint at address 0x%x %x.", virt_address.hi, virt_address.lo);
    dap_send_response((dap_response_t){ StatusOk, 0, 0 });
}

/** Remove a DAP code breakpoint */
static void dap_handle_remove_code_breakpoint(const uint64_t addr)
{
    const ptr64_t virt_address = { addr };
    alert("Removing DAP code breakpoint from address 0x%x %x.", virt_address.hi, virt_address.lo);

    rv_cpu_t* cpu = get_cpu(cpuno_global)->data;

    breakpoint_t *breakpoint = breakpoint_find_by_address(cpu->bps, virt_address, BREAKPOINT_FILTER_SIMULATOR);
    if (breakpoint == NULL) {
        alert("No such breakpoint!");
        dap_send_response((dap_response_t){ StatusUnspecifiedError, 0, 0 }); // TODO: more specific err code
        return;
    }

    list_remove(&cpu->bps, &breakpoint->item);
    safe_free(breakpoint)
    dap_send_response((dap_response_t){ StatusOk, 0, 0 });
}

void dap_process(void)
{
    dap_request_t request = { 0 };

    while (dap_receive_request(&request)) {
        switch (request.type) {
        case ResumeRequest:
            dap_handle_resume();
            continue;
        case SetCodeBreakpointRequest:
            dap_handle_set_code_breakpoint(request.arg0);
            continue;
        case RemoveCodeBreakpointRequest:
            dap_handle_remove_code_breakpoint(request.arg0);
            continue;
        default:
            alert("Unknown DAP request type %u.", request.type);
            dap_send_response((dap_response_t){ StatusUnsupportedRequestError, 0, 0 });
        }
    }
}
