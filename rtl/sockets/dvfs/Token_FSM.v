module Token_FSM (
    clock,  // NoC clock
    reset,  // Active high, synchronous reset
    packet_in,  // Received packet flag
    packet_in_val,  // Received packet value
    packet_out,  // Sending packet flag
    packet_out_val,  //Sent packet value
    packet_out_ready,  //Input to FSM indicating if the NoC is ready to accept a packet
    enable,  //FSM turned on, else token count not updated 
    packet_out_addr,  //Sent packet address 32 accelerator IDs addressable
    activity,  //Activity flag from the tile, 1b 
    max_tokens , //Configuration register (to sync from tile). Indicates max number of tokens that the tile can use at max F/V
    token_counter_override , //Register used to overwirte token counter from FSM if token_counter_override[10]==1
    tokens_next,  //Token counter next vlaue, to send to a read-only CSR in the tile
    packet_in_addr,  //Received packet address now 10b [4:0]: X coordinate [9:5] Y coordinate
    refresh_rate_min,  //From CSR to program refresh rate min value
    refresh_rate_max,  //From CSR to program refresh rate max value
    random_rate,  //From CSR, configures how often exchange happens with a random tile
    LUT_write,  //From CSR, used to wirte values in FSM LUT [17]=WEN [15:8]: value [7:0] addr
    LUT_read,  //To CSR value read from LUT wehn LUT_write[16]==1
    freq_target,  //Output of FSM, to send to LDO
    neighbors_ID,  //From CSR, specifies W/E/N/S neighbors
    PM_network  //From CSR lists which accelerators IDs are part of PM network
 //   thermal_overrun,      // NEW: flag from wrapper
   
);


    //-------------Input Ports-----------------------------
    input clock, reset, packet_in;  //Add enable input 
    input [31:0] packet_in_val;
    input [5:0] max_tokens;  //Unsigned
    input activity;
    input [4:0] packet_in_addr;
    input [11:0] refresh_rate_min;
    input [11:0] refresh_rate_max;
    input [4:0] random_rate;
    input packet_out_ready;
    input enable;
    input [17:0] LUT_write;
    input [7:0] token_counter_override;
    input [19:0] neighbors_ID;
    input [31:0] PM_network;
    input thermal_overrun;    // NEW
    //-------------Output Ports----------------------------
    output packet_out;
    output [31:0] packet_out_val;
    output [4:0] packet_out_addr;
    output [6:0] tokens_next;
    output [7:0] LUT_read;
    output [7:0] freq_target;
    //-------------Input ports Data Type-------------------
    wire clock, reset, packet_in;
    wire [31:0] packet_in_val;
    wire [5:0] max_tokens;
    wire activity;
    wire [4:0] packet_in_addr;
    wire [11:0] refresh_rate_min;
    wire [11:0] refresh_rate_max;
    wire [4:0] random_rate;
    wire packet_out_ready;
    wire enable;
    wire [17:0] LUT_write;
    wire [19:0] neighbors_ID;
    wire [31:0] PM_network;
    //-------------Output Ports Data Type------------------
    reg packet_out;
    reg [31:0] packet_out_val;
    reg [4:0] packet_out_addr;
    reg signed [6:0] tokens_next;
    reg [7:0] LUT_read;
    reg [7:0] freq_target;
    reg signed [6:0] token_counter;
    reg [31:0] PM_network_shifted;

   // -----------Token contributor tracking (ring buffer) ------------------------------------
    reg [4:0]  coin_src_addr   [3:0];
    reg [6:0]  coin_src_amount [3:0];
    reg [1:0]  coin_src_ptr;
    reg [4:0]  coin_src_addr_next   [3:0];
    reg [6:0]  coin_src_amount_next [3:0];
    reg [1:0]  coin_src_ptr_next;
    reg      valid_src       [3:0];  // Valid flag per contributor
    reg      valid_src_next  [3:0];

    // Pullback logic
    reg [3:0] pullback_count, pullback_count_next;
    reg       pull_triggered, pull_triggered_next;
    reg [1:0] pullback_index, pullback_index_next;
    reg       in_pullback_loop, in_pullback_loop_next;
//------------------------------------------------------------------------------------------
    //-------------Internal Constants--------------------------
    parameter SIZE_COUNT = 11;
    parameter SIDE_COUNT = 5;
    parameter SIZE_TOKEN = 7;
    //parameter COUNT_MIN = 10;
    //parameter COUNT_MAX = 2000; Used for dynamic diming

    parameter [6:0] RATIO_THRESHOLD = 7'd58; // ~90% of 64 max (scaled to 64)

    //-------------Internal Variables---------------------------
    reg         [SIZE_COUNT-1:0] refresh_count;
    reg         [SIZE_COUNT-1:0] refresh_count_next;
    reg         [SIZE_COUNT-1:0] side_count;
    reg         [SIZE_COUNT-1:0] side_count_next;
    reg         [          11:0] refresh_rate;
    reg         [          11:0] refresh_rate_next;
    reg         [           7:0] LUT                     [63:0];
    reg         [           7:0] LUT_next                [63:0];
    reg         [           7:0] freq_target_next;
    reg                          start_divider;
    wire        [          12:0] divider;
    wire                         sign;
    wire signed [           6:0] token_delta_div;
    wire        [          31:0] packet_out_val_div;
    wire        [           4:0] packet_out_addr_div;
    wire signed [          13:0] diva;
    wire signed [          13:0] divb;
    reg         [          31:0] PM_network_shifted_next;


 // New cooldown state
       reg [SIZE_COUNT-1:0] cooldown_cnt, cooldown_cnt_next; //new
       reg                  cool_off_flag, cool_off_flag_next; //new

    wire        [           5:0] max_tokens_act;
    reg                          freeze_div;
    wire signed [           6:0] zerozero;
    assign max_tokens_act = activity ? max_tokens : 0;
    //Divide runit implementation
    assign diva = $signed(packet_in_val[6:0]) * $signed({1'b0, max_tokens_act});
    assign divb = $signed(packet_in_val[16:10]) * token_counter;

    assign sign = (diva > divb) ? 1 : 0;  //+1 if need to receive tokens
    assign divider = (diva > divb) ? $unsigned(
            diva - divb
        ) : $unsigned(
            divb - diva
        );  //Convert to unsigned
    assign zerozero = ($signed(packet_in_val[6:0]) - token_counter) / 2;



    divider_unit DIV0 (
        .clock(clock),  // clock
        .rst(reset),  // Active high, syn reset
        .divider(divider),
        .divisor({1'b0, max_tokens_act} + {1'b0, packet_in_val[15:10]}),
        .packet_out(packet_out_div),  // Grant 0
        .packet_out_val(packet_out_val_div),
        .packet_out_addr(packet_out_addr_div),
        .packet_in_addr(packet_in_addr),
        .token_counter(token_counter),
        .flag_start(start_divider),
        .sign(sign),
        .freeze(freeze_div),
        .zerozero(zerozero),  //In case of 0/0
        .token_delta(token_delta_div)
    );
    //----------Reg Logic-----------------------------
    integer i;
    always @(posedge clock) begin : OUTPUT_LOGIC
        if (reset == 1'b0) begin
            refresh_count <= 0;
            side_count    <= 0;
            refresh_rate  <= 15;  //Arbitary value, will be updated by CSR write
            for (i = 0; i < 64; i = i + 1) begin
                LUT[i] <= 8'b0;
            end
            freq_target        <= 0;
            token_counter      <= 0;
            PM_network_shifted <= 0;
           
    //--------------------------------------
            coin_src_ptr         <= 0;
            pullback_count       <= 0;
            pull_triggered       <= 0;
            pullback_index       <= 0;
            in_pullback_loop     <= 0;
            
            for (i = 0; i < 4; i = i + 1) begin
                coin_src_addr[i]   <= 0;
                coin_src_amount[i] <= 0;
                valid_src[i]       <= 0;
            end      
            
    //------------------------------------        

        end else begin
            refresh_count      <= refresh_count_next;
            side_count         <= side_count_next;
            refresh_rate       <= refresh_rate_next;
            LUT                <= LUT_next;
            freq_target        <= freq_target_next;
            token_counter      <= tokens_next;
            PM_network_shifted <= PM_network_shifted_next;

    //-----------------------------------------------------
            coin_src_ptr       <= coin_src_ptr_next;
            pullback_count     <= pullback_count_next;
            pull_triggered     <= pull_triggered_next;
            pullback_index     <= pullback_index_next;
            in_pullback_loop   <= in_pullback_loop_next;

            for (i = 0; i < 4; i = i + 1) begin
                coin_src_addr[i]   <= coin_src_addr_next[i];
                coin_src_amount[i] <= coin_src_amount_next[i];
                valid_src[i]       <= valid_src_next[i];
            end
    //------------------------------------------------------        

        end
    end  // End Of Block OUTPUT_LOGIC

    always @* begin : COMBO
        //Combinational output

        //Default
        cooldown_cnt_next  = cooldown_cnt; //new
        cool_off_flag_next = cool_off_flag; //new
        side_count_next         = side_count;
        refresh_count_next      = refresh_count + 1;
        refresh_rate_next       = refresh_rate;
        start_divider           = 0;
        tokens_next             = token_counter;
        packet_out              = 0;
        packet_out_val          = 0;
        packet_out_addr         = 0;
        PM_network_shifted_next = PM_network_shifted;
        freeze_div              = 0;

//————————————————————————————————————————————————————————————————————————————————————————————————
        pullback_count_next     = pullback_count;
        pull_triggered_next     = pull_triggered;
        pullback_index_next     = pullback_index;
        in_pullback_loop_next   = in_pullback_loop;
        
        for (i = 0; i < 4; i = i + 1) begin
            coin_src_addr_next[i]   = coin_src_addr[i];
            coin_src_amount_next[i] = coin_src_amount[i];
            valid_src_next[i]       = valid_src[i];
        end
        coin_src_ptr_next = coin_src_ptr;
// ————————————————————————————————————————————————————————————————————————————————————————————————————
        coin_src_addr_next[coin_src_ptr]   = packet_in_addr;
        coin_src_amount_next[coin_src_ptr] = packet_in_val[6:0];
      //  coin_src_ptr_next                  = coin_src_ptr + 1;
        coin_src_ptr_next                  = (coin_src_ptr + 1) & 2'b11; // Wrap pointer

       if (packet_in == 1 && packet_in_val[31] == 0 && enable == 1) begin  //Received update
            tokens_next = token_counter + $signed(packet_in_val[6:0]);
            valid_src_next[coin_src_ptr] = 1;
//----------------------------------------
        

            // Pullback trigger logic based on token utilization
        if (enable && max_tokens != 0 && token_counter[6] == 0) begin
            if ((token_counter <<< 6) / max_tokens > RATIO_THRESHOLD) begin
                if (!pull_triggered) begin
                    pullback_count_next = 5;  // wait 5 cycles
                    pull_triggered_next = 1;
                end else if (pullback_count != 0) begin
                    pullback_count_next = pullback_count - 1;
                end
            end else begin
                pullback_count_next = 0;
                pull_triggered_next = 0;
            end
        end
       // Pullback Loop: Send Return Tokens
        if (pull_triggered && pullback_count == 0) begin
            in_pullback_loop_next = 1;
            pullback_index_next   = 0;
            pull_triggered_next   = 0;
        end
        
        if (in_pullback_loop) begin
            if (valid_src[pullback_index]) begin
                // (send packet)
            end
            if (packet_out_ready && enable && valid_src[pullback_index]) begin
                packet_out        = 1;
                packet_out_addr   = coin_src_addr[pullback_index];
                packet_out_val[31] = 0;  // Data packet
                packet_out_val[6:0] = ~coin_src_amount[pullback_index] + 1;  // Two’s complement
                tokens_next       = token_counter - coin_src_amount[pullback_index];
        
                if (pullback_index == 3)
                    in_pullback_loop_next = 0;
                else
                    pullback_index_next = pullback_index + 1;
            end
                // Clear after use
                valid_src_next[pullback_index]       = 0;
                coin_src_amount_next[pullback_index] = 0;
        end
            // Skip rest of FSM while returning tokens
                  disable COMBO;
            end
        
//------------------------------------------            

            if (packet_in_val[6:0] == 0) begin
                if ((refresh_rate + refresh_rate >> 1) <= refresh_rate_max)
                    refresh_rate_next = refresh_rate + refresh_rate >> 1;  //x1.5
                else refresh_rate_next = refresh_rate_max;
            end else begin
                if ((refresh_rate >> 2 + refresh_rate[1]) >= refresh_rate_min)
                    refresh_rate_next = refresh_rate >> 2 + refresh_rate[1];  //x0.25		
                else refresh_rate_next = refresh_rate_min;
            end
        end

        if (packet_in==1 && packet_in_val[31]==1 && enable==1) begin //Received status, start the divider pipeline
            //if(refresh_count>refresh_rate-2)
            //	refresh_count_next=refresh_rate-2;// to avoid collision between refresh and compute update calculation
            start_divider = 1;
            freeze_div    = 0;
        end

        if (packet_out_div==1 && packet_out_ready==1 && enable==1) begin //Send update, NoC ready
            if (packet_in == 1 && packet_in_val[31] == 0 && enable == 1)
                tokens_next = token_counter + token_delta_div + $signed(
                    packet_in_val[6:0]
                );  //Apply both updates at once
            else tokens_next = token_counter + token_delta_div;
            packet_out      = packet_out_div;
            packet_out_val  = packet_out_val_div;
            packet_out_addr = packet_out_addr_div;
        end

        if (packet_out_div==1 && packet_out_ready==0 && enable==1) begin //Freeze state till NoC ready to receive 
            freeze_div = 1;
        end

        if (packet_out == 1 && packet_out_val == 0) begin  //Update refresh rates
            if ((refresh_rate + refresh_rate >> 1) <= refresh_rate_max)
                refresh_rate_next = refresh_rate + refresh_rate >> 1;  //x1.5
            else refresh_rate_next = refresh_rate_max;
        end else begin
            if ((refresh_rate >> 2 + refresh_rate[1]) >= refresh_rate_min)
                refresh_rate_next = refresh_rate >> 2 + refresh_rate[1];  //x0.25		
            else refresh_rate_next = refresh_rate_min;
        end

        if (packet_in==0 && refresh_count>refresh_rate && packet_out_ready==1 && enable==1 && packet_out_div==0) begin //Send status with NoC ready to accept
            packet_out = 1;
            if ((side_count_next + 1 >= random_rate) && (random_rate != 0)) begin
                packet_out_addr=PM_network_shifted&~(PM_network_shifted-1); // Selects the 1st 1 https://medium.com/@manishsakariya/finding-position-of-first-least-significant-non-zero-bit-in-binary-number-6df144602f89
                side_count_next = 0;
                PM_network_shifted_next=((PM_network_shifted&(PM_network_shifted-1))==0)?PM_network:(PM_network_shifted&(PM_network_shifted-1));//Removes the 1st one from PM_network_shifted
            end else begin
                packet_out_addr=(neighbors_ID&(5'b11111<<(5*side_count[1:0])))>>(5*side_count[1:0]);
                side_count_next = side_count + 1;
            end
            packet_out_val[31]    = 1;  //Status type of packet
            packet_out_val[19:10] = max_tokens_act;  //Needs
            packet_out_val[6:0]   = token_counter;  //Has
            refresh_count_next    = 0;
        end  //end if

        //LUT definitons, read from internal LUT
        if (LUT_write[16] == 1) LUT_read <= LUT[LUT_write[7:0]];
        else LUT_read <= 0;
        //Write to internal LUT
        LUT_next <= LUT;
        if (LUT_write[17] == 1) LUT_next[LUT_write[7:0]] <= LUT_write[15:8];


        if (token_counter_override[7] == 1) tokens_next = token_counter_override[6:0];

        if (tokens_next[6] == 0)  //posivite
            freq_target_next <= LUT[tokens_next[5:0]];
        else  //Min value
            freq_target_next <= LUT[0];
    end  //End Combo

endmodule  // End of Module
