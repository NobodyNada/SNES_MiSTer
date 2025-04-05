/* UART driver for Super Nintendo Interface connectivity */

module sni(
    input wire          clk,
    input wire          reset,
    input wire          vblank,

    output wire[16:0]   wram_addr,
    output wire[7:0]    wram_data,
    input wire[7:0]     wram_q,
    output wire         wram_wren,
    
    // Set to 1 when WRAM starts processing our request, and 0 once it finishes
    input wire          wram_busy,

    output wire         rbf,
    input wire          txint,
    input wire          rxint,
    output reg          tdata_i,
    output wire [15:0]  tdata_m,
    input wire [15:0]   rdata_m
    );

    reg [8:0] inbuffer_raddr;
    reg [8:0] inbuffer_waddr;
    wire [7:0] inbuffer_rdata;
    wire [7:0] inbuffer_wdata;

    // recieve buffer is full if head = tail - 1
    assign rbf = (inbuffer_waddr == inbuffer_raddr - 1);

    // State machine for UART interface
    enum bit[3:0] {
        // command-parsing states
        STATE_CMD       = 4'h0,
        STATE_ADDR      = 4'h1,     // 1, 2, 3
        STATE_LEN       = 4'h4,     // 4,
        STATE_PING      = 4'h5,     // 5
        
        // command-responding states
        STATE_TRANSFER  = 4'h8,	    // 8, 9
		STATE_WAITNMI   = 4'hA      // A
  
    } State;
    reg[3:0] state;

    reg last_vblank;
    reg[23:0] addr;
    wire wram_sel = (addr[23:16] == 8'hF5 || addr[23:16] == 8'hF6);
    assign wram_addr = {~addr[16], addr[15:0]};
    
    // Whether we've seen wram_busy go high for our current request.
    reg wram_started;
    reg last_wram_busy;
    
    // Becomes 1 once our WRAM request finishes.
    wire wram_finished;
    assign wram_finished = ~wram_sel | (wram_started & ~wram_busy);

    reg[7:0] len;

    enum bit[7:0] {
        CMD_PING  	= 8'h00,
        CMD_READ    = 8'h1,
        CMD_WRITE   = 8'h2,
        CMD_WAITNMI = 8'h3
    } Cmd;
    reg rw;

    // UART RX buffer
    dpram #(9) inbuffer
    (
        .clock(~clk),
        .address_a(inbuffer_waddr),
        .data_a(inbuffer_wdata),
        .wren_a(1'b1),

        .address_b(inbuffer_raddr),
        .q_b(inbuffer_rdata),
        .wren_b(1'b0)
    );

    assign inbuffer_wdata = rdata_m[7:0];

    reg[7:0] tdata;
    assign tdata_m = {8'h01, tdata};

    reg last_txint;
    reg last_rxint;
    reg tinprogress;

    always @(posedge clk) begin
        last_txint <= txint;
        last_rxint <= rxint;
        last_wram_busy <= wram_busy;
        tdata_i <= 1'b0;
        wram_wren <= 1'b0;
        last_vblank <= vblank;

        if (reset) begin
            inbuffer_raddr <= 9'h000;
            inbuffer_waddr <= 9'h000;
            tinprogress <= 1'b0;
            last_vblank <= 1'b0;
            wram_started <= 1'b0;
            state <= STATE_CMD;
        end else begin
            if (!rxint && last_rxint) begin
                // We've recieved data, update the write address for the next byte
                inbuffer_waddr <= inbuffer_waddr + 1'b1;
            end
            if (tinprogress) begin
                // If we're currently transmitting, don't do anything until the transmission finishes.
                if (!txint && last_txint) tinprogress <= 1'b0;
            end else if (state[3] == 0) begin
                // If we're in a command-parsing state, and we have a data byte available...
                if (inbuffer_raddr != inbuffer_waddr) begin
                    // Automatically advance to the next state unless the specific state overrides this behavior.
                    state <= state + 1'b1;
                    // Advance the read buffer index.
                    inbuffer_raddr <= inbuffer_raddr + 1'b1;
                    case (state)
                        STATE_CMD: begin
                            case (inbuffer_rdata)
                                CMD_PING: begin
                                    // Respond with a length of 1, then read the next data byte
                                    tdata_i <= 1'b1;
                                    tinprogress <= 1'b1;
                                    tdata <= 1'b1;
                                    state <= STATE_PING;
                                end
                                CMD_READ: rw <= 0;  // go to STATE_ADDR
                                CMD_WRITE: rw <= 1; // go to STATE_ADDR
                                CMD_WAITNMI: state <= STATE_WAITNMI;
                                
                                // Invalid command, ignore
                                default: state <= STATE_CMD;
                            endcase
                        end
                        STATE_ADDR+0: addr[7:0]   <= inbuffer_rdata;
                        STATE_ADDR+1: addr[15:8]  <= inbuffer_rdata;
                        STATE_ADDR+2: addr[23:16] <= inbuffer_rdata;
                        STATE_LEN+0: begin
                            len <= inbuffer_rdata;
                            tdata_i <= 1'b1;
                            tinprogress <= 1'b1;
                            state <= STATE_TRANSFER;
                            wram_started <= 0;
                            if (rw) begin
                                // write, response length is 0
                                tdata <= 8'b0;
                            end else begin
                                // read, response length is the requested length
                                tdata <= inbuffer_rdata;
                            end
                        end
                        STATE_PING: begin
                            // Echo the data byte we recieved.
                            tdata_i <= 1'b1;
                            tinprogress <= 1'b1;
                            tdata <= inbuffer_rdata;
                            state <= STATE_CMD;
                        end
                    endcase
                end
            end else if (state == STATE_TRANSFER || state == STATE_TRANSFER+1) begin
                if (~last_wram_busy & wram_busy) wram_started <= 1'b1;
                // If length is 0, we're done
                if (len == 24'b0) state <= STATE_CMD;
                else if (rw) begin
                    // Write
                    if (inbuffer_raddr != inbuffer_waddr) begin
                        // If we have data available, write it to memory.
                        if (wram_sel) begin
                            wram_wren <= 1'b1;
                            wram_data <= inbuffer_rdata;
                        end
                        if (wram_finished) begin
                            wram_started <= 0;
                            len <= len - 1'b1;
                            // Don't increment the address until the second byte.
                            if (state == STATE_TRANSFER+0) begin
                                state <= STATE_TRANSFER+1;
                            end else begin
                                addr <= addr + 24'b1;
                            end
                        end
                    end
                end else if (wram_finished) begin
                    // This is a read, echo some data from WRAM
                    tdata <= wram_sel ? wram_q : 8'b0;
                    tinprogress <= 1'b1;
                    tdata_i <= 1'b1;
                    len <= len - 1'b1;
                    addr <= addr + 24'b1;
                    wram_started <= 0;
                end
            end else if (state == STATE_WAITNMI) begin
                if (vblank && ~last_vblank) begin
                    // Response length = 0
                    tinprogress <= 1'b1;
                    tdata_i <= 1'b1;
                    tdata <= 8'b0;
                    state <= STATE_CMD;
                end
            end
        end
    end
endmodule
