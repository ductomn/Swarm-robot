#include "state_machine.h"

// Communication
adc1_channel_t adc1_channels[] = {IO_SIG_FRONT, IO_SIG_FRONT_RIGHT};
adc2_channel_t adc2_channels[] = {IO_SIG_BACK_RIGHT, IO_SIG_BACK, IO_SIG_BACK_LEFT, IO_SIG_FRONT_LEFT};
gpio_num_t led_sig[] = {IO_IR_SIG};
int led_sig_num = GET_SIZE(led_sig);

// Obstacle sensing
adc1_channel_t dis_channels[] = {IO_DIS_LEFT, IO_DIS_RIGHT};
gpio_num_t led_dis[] = {IO_IR_DIS};
int led_dis_num = GET_SIZE(led_dis);

static CommState comm_state = IDLE;

static int send = 1;               // chosen command to send
static int send_num = 0;           // current sent repetition
static int rx_msg[CHANNEL_NUM];    // decoded message

static int command1 = 0;           // command count
static int command2 = 0;
static int command3 = 0;

static bool leader = 0;
static bool leader_reset = 0;      // flag to delay after being leader

static uint32_t time_now = 0;
static uint32_t timer_command = 0;

// Commands
static uint8_t direction = 0;
static int adc_results[CHANNEL_NUM];

static bool cmd_close_enough = 0;

static bool cmd2_return = 0;


// delay after turning on
static int wait_time;
static int start_flag = 0;
static bool detect = 0;


void state_machine_loop() {

    while (1)
    {
        time_now = hwtimer_get_time();
        timer_command = hwtimer_cmd_get_time();

        switch (comm_state)
        {
            case IDLE:
                state_idle();
                break;
            case RANDOM_WALK:
                state_random_walk();
                break;
            case LISTEN:
                state_listen();
                break;
            case TRANSMITTING:
                state_transmitting();
                break;
            case COMMAND_RECEIVED:
                state_command_received();
                break;
            case COMMAND1:
                state_command1();
                break;
            case COMMAND2:
                state_command2();
                break;
            case COMMAND3:
                state_command3_chain();
                break;


            case CHAIN_FORMATION:
                state_chain();  
                break;          
        }

        #if !CHAIN
        state_command_clear();
        
        obstacle_avoidance();
        #endif
    }
    
}

void state_machine_init() {
    // Initialize the communication module
    dm_comm_init(adc1_channels, GET_SIZE(adc1_channels), adc2_channels, GET_SIZE(adc2_channels), led_sig, led_sig_num);
    hwtimer_clock_init(1000000, BIT_DURATION_US);

    servo_init(SERVO_LEFT_CHANNEL, SERVO_LEFT_GPIO);
    servo_init(SERVO_RIGHT_CHANNEL, SERVO_RIGHT_GPIO);

    coop_dis_init(dis_channels, GET_SIZE(dis_channels), led_dis, led_dis_num);

    srand(esp_random()); // for True RNG
    wait_time = (rand() % RAND_IDLE_TIME) + MIN_IDLE_TIME;

    time_now = hwtimer_get_time();
    printf("\n%" PRIu32 " wait for %d", time_now, wait_time);
}



void state_idle() {
    if ((time_now >= wait_time)){
        #if !CHAIN
        
            #if !FOLLOW_CHAIN
        comm_state = RANDOM_WALK;
        wait_time = (rand() % RAND_WALK_TIME) + MIN_WALK_TIME;
        printf("\n%" PRIu32 " Start random walk for %d", time_now, wait_time);
        random_walk_start();
            #else

        comm_state = COMMAND3;
            #endif

        hwtimer_cmd_reset_clock();
        timer_command = hwtimer_cmd_get_time();
        hwtimer_reset_clock();
        time_now = hwtimer_get_time();

        #else

        comm_state = CHAIN_FORMATION;
        printf("\n%" PRIu32 " Start chain formation", time_now);

        #endif
    }
}


void state_random_walk() {
    random_walk_loop(timer_command, comm_state);

    printf("\n%" PRIu32 " Random Walk", time_now);

    #if WITH_LEADER
        #if !LEADER
        detect = dm_comm_detect_start_sig();

        if(detect){
            printf("\n Signal detected");
            comm_state = LISTEN;
            // signal_correction();
            hwtimer_reset_clock();
            time_now = 0;
        }

        // Check twice, just in case
        if(dm_comm_detect_start_sig()){
            printf("\n Signal detected");
            comm_state = LISTEN;
            // signal_correction();
            hwtimer_reset_clock();
            time_now = 0;
        }
        #else


        if ((time_now >= wait_time)){
            printf("\n Start transmitting");
            comm_state = TRANSMITTING;
            hwtimer_reset_clock();
            time_now = 0;
        }
        #endif
    #else

        detect = dm_comm_detect_signals();

        if(detect){
            printf("\n Signal detected");
            comm_state = LISTEN;
            // signal_correction();
            hwtimer_reset_clock();
            time_now = 0;
        }

        // Check twice, just in case
        if(dm_comm_detect_signals()){
            printf("\n Signal detected");
            comm_state = LISTEN;
            // signal_correction();
            hwtimer_reset_clock();
            time_now = 0;
        }


        if ((time_now >= wait_time)){
            printf("\n Start transmitting");
            comm_state = TRANSMITTING;
            hwtimer_reset_clock();
            time_now = 0;
        }

    #endif
}

void state_listen() {

    if (dm_comm_process()) {

        servo_stop();
        
        printf("\n%" PRIu32 "   1: %d \t 2: %d\n\n", time_now, command1, command2);

        dm_comm_get_messages(rx_msg);
        
        for (int i = 0; i < CHANNEL_NUM; i++) {
            // printf("Received: %d from channel %d\n", rx_msg[i], i);
            if(rx_msg[i] == CMD_START_SIG) {
                comm_state = COMMAND_RECEIVED;
                printf("\n  %" PRIu32 "   Command received\n", time_now);
            }
            if (rx_msg[i] == COMMAND1_SIG) command1++;
            if (rx_msg[i] == COMMAND2_SIG) command2++;
            if (rx_msg[i] == COMMAND3_SIG) command3++;
        }

        hwtimer_reset_clock();
        time_now = 0;

        //printf("\n\n");
    } else {

        // Spin?
        if ((time_now >= MSG_TIME_TAKEN))  servo_rotate_right(60);

        if ((time_now >= LISTEN_TIME)){
            comm_state = RANDOM_WALK;
            wait_time = (rand() % RAND_WALK_TIME) + MIN_WALK_TIME;
            printf("\n Signal not received");
            printf("\n%" PRIu32 " Start random walk for %d", time_now, wait_time);
            command1 = 0;
            command2 = 0;
            command3 = 0;
            printf("\nReset clock");
            hwtimer_reset_clock();
            time_now = 0;

            random_walk_start();
            hwtimer_cmd_reset_clock();
            timer_command = hwtimer_cmd_get_time();
        }
    }
}

void state_transmitting() {

    servo_stop();

    if (!dm_comm_backoff())
    {
        if (time_now >= MSG_INTERVAL)
        {

            if (leader_reset)
            {
                // dm_comm_stop();
                printf("\n Leader reset");
                // dm_comm_set_backoff((COMMAND_PERIOD + LEADER_BACKOFF));
                leader_reset = 0;
                if (leader)
                {
                    if (send == 1)
                    {
                        comm_state = COMMAND1;
                        // random_walk_start();
                        hwtimer_cmd_reset_clock();
                        timer_command = hwtimer_cmd_get_time();
                        dm_comm_reading_stop();
                    }
                    if (send == 2)
                    {
                        comm_state = COMMAND2;
                        servo_stop();
                        hwtimer_cmd_reset_clock();
                        timer_command = hwtimer_cmd_get_time();
                        dm_comm_reading_stop();
                    }
                    if (send == 3)
                    {
                        comm_state = COMMAND3;
                        // random_walk_start();
                        hwtimer_cmd_reset_clock();
                        timer_command = hwtimer_cmd_get_time();
                        // dm_comm_reading_stop();
                    }
                }
                else
                {
                    comm_state = RANDOM_WALK;
                    random_walk_start();
                    hwtimer_cmd_reset_clock();
                    timer_command = hwtimer_cmd_get_time();
                    wait_time = (rand() % RAND_WALK_TIME) + MIN_WALK_TIME;
                    printf("\n%" PRIu32 " Start random walk for %d", time_now, wait_time);
                }
                return;
            }

            printf("\n%" PRIu32 "\t%d \tTransmitting   %d", time_now, send_num, send);
            
            if (send_num++ == 0) send = (rand() % 2) + 1;

            if (send == 1) dm_comm_send(COMMAND1_SIG);
            if (send == 2) dm_comm_send(COMMAND2_SIG);
            if (send == 3) dm_comm_send(COMMAND3_SIG);

            if (send_num >= MAX_SEND_COUNT)
            {
                send_num = 0;
                dm_comm_send(CMD_START_SIG);
                leader_reset = 1;
                leader = 1;
                printf("\n %" PRIu32 " Done sending %d", time_now, send);

                // comm_state = LISTEN;  // RANDOM_WALK;
            }

            hwtimer_reset_clock();
            time_now = 0;
        }
    }
    else
    {
        send_num = 0;
        comm_state = RANDOM_WALK;
        random_walk_start();
        hwtimer_cmd_reset_clock();
        timer_command = hwtimer_cmd_get_time();
        printf("\nChannel occupied");
        wait_time = (rand() % RAND_WALK_TIME) + MIN_WALK_TIME;
        printf("\n%" PRIu32 " Start random walk for %d", time_now, wait_time);
    }
}

void state_command_received() {
    if (command1 >= COMMAND_COUNT) {
        // dm_comm_stop();
        dm_comm_reading_stop();
        hwtimer_cmd_reset_clock();
        timer_command = hwtimer_cmd_get_time();
        command1 = 0;
        command2 = 0;
        command3 = 0;
        printf("Commencing command 1\n");
        comm_state = COMMAND1;
        hwtimer_reset_clock();
    }

    else if (command2 >= COMMAND_COUNT) {
        //dm_comm_stop();
        dm_comm_reading_stop();
        hwtimer_cmd_reset_clock();
        timer_command = hwtimer_cmd_get_time();
        coop_start_spread();
        command1 = 0;
        command2 = 0;
        command3 = 0;
        printf("%" PRIu32 " Commencing command 2\n", time_now);
        comm_state = COMMAND2;
        hwtimer_reset_clock();
    }

    else if (command3 >= COMMAND_COUNT) {
        // dm_comm_stop();
        // dm_comm_reading_stop();
        hwtimer_cmd_reset_clock();
        timer_command = hwtimer_cmd_get_time();
        command1 = 0;
        command2 = 0;
        command3 = 0;
        printf("Commencing command 1\n");
        comm_state = COMMAND3;
        hwtimer_reset_clock();
    }

    else comm_state = LISTEN;

    time_now = hwtimer_get_time();
}

void state_command1() {
    if(leader){
        multiple_led_drive(led_sig, led_sig_num, 1);
        random_walk_loop(timer_command, comm_state);
        // servo_move_forward(300);
        printf("\n%" PRIu32 " 1: Leader", time_now);
    } else {

        dm_comm_get_signals(adc_results);
        direction = coop_signal_direction(adc_results);
        printf("%" PRIu32 " \tDirection: %d\n\n",time_now, direction);
        coop_turn_to_signal(direction);
        if (cmd_close_enough) servo_stop();
        else servo_move_forward(SERVO_MOVE_SPEED);
        printf("\n%" PRIu32 " 1: Rotating right", time_now);

    }
}

void state_command2() {
    if(leader){
        if ((time_now <= TURN_AWAY_TIME_US + 500) || (time_now >= (TURN_AWAY_TIME_US + FORWARD_TIME_US + REVERSE_TIME_US + FORWARD_TIME_US - 500)))
        {
            multiple_led_drive(led_sig, led_sig_num, 1);
        } else multiple_led_drive(led_sig, led_sig_num, 0);
        
        
        printf("\n%" PRIu32 " 2: Leader", time_now);
    } else {

        coop_spread_out_loop(time_now, cmd_close_enough);
        if (coop_get_state() == STATE_DONE) cmd2_return = 1;

        //servo_rotate_left(100);
        //printf("\n%" PRIu32 " 2: Rotating left", time_now);
    }
}
 


// What about skipping RANDOM_WALK and go into state COMMAND3 immediately?
// %%%%%%%%%%%   ID hard coded   %%%%%%%%%%%%%%

void state_command3_chain() {
    if (leader || (WITH_LEADER && ROBOT_ID==1)) {
        if (timer_command >= MSG_INTERVAL) {
            dm_comm_send(1);  // Leader sends "1"
            hwtimer_cmd_reset_clock();
            timer_command = 0;
        }
        random_walk_loop(time_now, comm_state);
        printf("\n%" PRIu32 " 1: CHAIN Leader", time_now);
        return;
    }

    // ID assigned — follow the one before
    if (dm_comm_process()) {
        dm_comm_get_messages(rx_msg);
        for (int i = 0; i < CHANNEL_NUM; i++) {
            if (rx_msg[i] == (ROBOT_ID-1))  {
                printf("\nTarget: %d   Direction: %d", ROBOT_ID-1, i);
                coop_turn_to_signal(i);
                
                // dm_comm_get_msg_strength(adc_results);
                
                // if (cmd_close_enough) {
                //     servo_stop();
                //     cmd_close_enough = 0;
                // }
                // else servo_move_forward(SERVO_MOVE_SPEED);

                if (cmd_close_enough) servo_stop();
                else servo_move_forward(SERVO_MOVE_SPEED+100);

                break;
            }
        }
    }

    // Send your own ID so next robot can follow you
    if (timer_command >= MSG_INTERVAL) {
        dm_comm_send(ROBOT_ID);
        hwtimer_cmd_reset_clock();
        timer_command = 0;
    }
}




// %%%%%%%%%%%   Dynamic ID assignment ---- NOT WORKING IDEALLY   %%%%%%%%%%%%%
// static uint8_t my_id = -1;
// static uint8_t target_id = -1;
// static bool id_locked = false;  // Prevent changing ID again

// void state_command3_chain() {
//     if (leader) {
//         if (timer_command >= MSG_INTERVAL)) {
//             dm_comm_send(1);  // Leader sends "1"
//             hwtimer_cmd_reset_clock();
//             timer_command = 0;
//         }
//         random_walk_loop(time_now, comm_state);
//         printf("\n%" PRIu32 " 1: CHAIN Leader", time_now);
//         return;
//     }

//     // If ID not assigned, listen
//     if (!id_locked) {
//         int highest_id = -1;
//         int direction_of_highest = -1;

//         while (timer_command <= (ROBOT_ID * 500 + (ROBOT_ID-2)*700)){
//             timer_command = hwtimer_cmd_get_time();
//             printf("\n Waiting  %" PRIu32 "...", timer_command);
//         }
        
//         int sample = 0;
//         while (sample < 50){
//             if (dm_comm_process()) {
//                 dm_comm_get_messages(rx_msg);
    
//                 for (int i = 0; i < CHANNEL_NUM; i++) {
//                     if (rx_msg[i] > highest_id && rx_msg[i] > 0 && rx_msg[i] < 4) {
//                         highest_id = rx_msg[i];
//                         direction_of_highest = i;
//                     }
//                 }

//                 sample++;
//             }
//         }
    
//         if (highest_id > 0) {
//             target_id = highest_id;
//             my_id = highest_id + 1;
//             id_locked = true;
    
//             printf("\nROBOT %d: Locking to ID %d → My ID = %d", ROBOT_ID, target_id, my_id);
//             coop_turn_to_signal(direction_of_highest);
//         }
    
//     }
    

//     // ID assigned — follow the one before
//     if (id_locked && target_id != -1) {
//         if (dm_comm_process()) {
//             dm_comm_get_messages(rx_msg);
//             for (int i = 0; i < CHANNEL_NUM; i++) {
//                 if (rx_msg[i] == target_id) {
//                     printf("\nTarget: %d   Direction: %d", target_id, i);
//                     coop_turn_to_signal(i);
//                     if (cmd_close_enough) servo_stop();
//                     else servo_move_forward(300);
//                     break;
//                 }
//             }
//         }

//         // Send your own ID so next robot can follow you
//         if (timer_command >= MSG_INTERVAL)) {
//             dm_comm_send(my_id);
//             hwtimer_cmd_reset_clock();
//             timer_command = 0;
//         }
//     }
// }



void state_command_clear() {
    if(comm_state >= COMMAND1 && (time_now  >= (COMMAND_PERIOD)) && comm_state != COMMAND3) {
        // dm_comm_start();                 // well... dm_comm_stop and dm_comm_start crashes, not needed I guess
        // vTaskDelay(pdMS_TO_TICKS(10));   // ??? without delay crashes, crashes even with, but later (after 2. or 3. iteration) ???
        multiple_led_drive(led_sig, led_sig_num, 0);
        dm_comm_reading_start();
        servo_stop();          
        printf("\n Command stopped");
        comm_state = RANDOM_WALK;
        if(leader) wait_time = (rand() % RAND_WALK_TIME) + MIN_WALK_TIME + LEADER_BACKOFF;
        else wait_time = (rand() % RAND_WALK_TIME) + MIN_WALK_TIME;
        leader = 0;
        random_walk_start();
        //printf("\nReset clock");
        hwtimer_cmd_reset_clock();
        timer_command = hwtimer_cmd_get_time();
        cmd2_return = 0;
        printf("\n%" PRIu32 " Start random walk for %d", time_now, wait_time);
        hwtimer_reset_clock();
        time_now = 0;
    }
}



void obstacle_avoidance() {
    if (coop_obstacle_detection()) {
        printf("\n Obstacle detected");
        if ((comm_state == COMMAND1 && !leader) || (comm_state == COMMAND2 && cmd2_return) || (comm_state == COMMAND3 && !(leader || (WITH_LEADER && ROBOT_ID==1)))) { 
            // if (comm_state != COMMAND3) dm_comm_get_signals(adc_results);
            
            // if (comm_state == COMMAND3) {
            //     int max = 0;
            //     for (int i = 0; i >= CYCLE_BIT_COUNT/2; i++) {
            //         dm_comm_get_signals(adc_results);
            //         if (max <= adc_results[0]) max = adc_results[0];
            //     }
            //     adc_results[0] = max;
            // }
            
            dm_comm_get_signals(adc_results);
            if (adc_results[0] >= (SIG_THRESHOLD + 3500)) {
                servo_move_backwards(500);
                vTaskDelay(pdMS_TO_TICKS(200));
                servo_stop();
                vTaskDelay(pdMS_TO_TICKS(10));
            } 
            else if (adc_results[0] >= (SIG_THRESHOLD + 3200)){
                cmd_close_enough = 1;
                // servo_rotate_right_91();     // is it needed?
            }
            else cmd_close_enough = 0;
        } else {
            servo_rotate_right_91();
            if (leader || (WITH_LEADER && ROBOT_ID==1)) servo_move_forward(SERVO_MOVE_SPEED);
            else servo_stop();
            //if (comm_state == COMMAND2) // log movement
        }
    }
}