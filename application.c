#include "application.h"
#include "stm32l5xx_hal.h"

#if APP_SEL == BASIC

void (*some_func)() = &func;

int func2(long some_data1, long some_data2){
    SECURE_new_log_entry();
    if(some_data2 > some_data1){
        return some_data2 - some_data1;    
    } else {
        return some_data1 - some_data2;    
    }
}

void func(int some_data){
    if(some_data % 4 == 0){
        some_data += some_data + func2(some_data, some_data);
    } else{
        some_data = some_data - func2(some_data, some_data);
    }
    int i = 0;
    while(i < 16){
        some_data = (some_data ^ 0xaa) << 1;
        i++;
    }
}

void test_application(){
    int some_data = 1234;

    func(some_data);

    some_func();
}

#endif

/******************* SYRINGE APP *********************/

#if APP_SEL == SYRINGE

//syringe movement direction
enum{PUSH,PULL};
int steps;

void delayMicroseconds(unsigned int delay)
{
    //SECURE_new_log_entry();
    volatile unsigned int j = 0;
    do{
        j++;
    } while(j <= delay);
}

char getserialinput(uint8_t inputserialpointer)
{
    //SECURE_new_log_entry();
    uint8_t maxinputpointer = 2;
    char input[2] = "+\n";
    if (inputserialpointer < maxinputpointer)
    {
        return input[inputserialpointer];
    }
    return 0;
}

void run_syringe()
{
    //SECURE_new_log_entry();
    /* -- Global variables -- */
    // Input related variables
    volatile uint8_t inputserialpointer = -1;
    uint16_t inputStrLen = 0;
    char inputStr[10]; //input string storage

    // Bolus size
    uint16_t mLBolus =  5;

    // Steps per ml
    int ustepsPerML = (MICROSTEPS_PER_STEP * STEPS_PER_REVOLUTION * SYRINGE_BARREL_LENGTH_MM) / (SYRINGE_VOLUME_ML * THREADED_ROD_PITCH );

    //int ustepsPerML = 10;
    int inner = 0;
    int outer = 0;
    steps = 0;

    while(outer < 1)
    {
       char c = getserialinput(++inputserialpointer);
       // hex to char reader
       while (inner < 10)
       {
   
           if(c == '\n') // Custom EOF
           {
       
               break;
           }
   
           if(c == 0)
           {
       
               outer = 10;
               break;
           }
   
           inputStr[inputStrLen++] = c;
           c = getserialinput(++inputserialpointer);
   
           inner += 1;
       }
       inputStr[inputStrLen++] = '\0';
       steps = mLBolus * ustepsPerML;

       for(int i=0; i < steps; i++)
       {
           if(inputStr[0] == '+' || inputStr[0] == '-')
           {
                // write 0xff to port
                sensor = 0xff;
                delayMicroseconds(SPEED_MICROSECONDS_DELAY);
            }
            // write 0x00 to port
            sensor = 0x00;
            delayMicroseconds(SPEED_MICROSECONDS_DELAY);
        }
        delayMicroseconds(SPEED_MICROSECONDS_DELAY);
        inputStrLen = 0;
        outer += 1;
    }
}

void application()
{
//SECURE_new_log_entry();
    int i ;
    for(i=0; i< 3; i++)
        run_syringe();
    SECURE_record_output_data(1);
}
#endif

#if APP_SEL == TEMP
int temp;
int humidity;
uint8_t data[5] = {0,0,0,0,0};
uint8_t valid_reading = 0;

extern __IO uint32_t uwTick;
void delay(unsigned int us){
    //SECURE_new_log_entry();
    int i = uwTick;
    while(i+us>=uwTick);
}

void read_data(){
    uint8_t counter = 0;
    uint16_t j = 0, i;

    // pull signal high & delay
    GPIOA->BRR = (uint32_t)GPIO_PIN_8;
    delay(250);

    /// pull signal low for 20us
    GPIOA->BSRR = (uint32_t)GPIO_PIN_8;
    delay(20);

    // pull signal high for 40us
    GPIOA->BRR = (uint32_t)GPIO_PIN_8;
    delay(40);

    //read timings
    for(i=0; i<MAX_READINGS; i++){

        counter += (GPIOA->IDR & GPIO_PIN_8) >> 8;

        // ignore first 3 transitions
        if ((i >= 4) && ( (i & 0x01) == 0x00)) {
    
            // shove each bit into the storage bytes
            data[j >> 3] <<= 1;
            if (counter > 6){
        
                data[j >> 3] |= 1;
            }
    
            j++;
        }

    }
    // check we read 40 bits and that the checksum matches
    if ((j >= 40) && (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {

        valid_reading = 1;
    } else {

        valid_reading = 0;
    }
}

uint16_t get_temperature(){
    read_data();

    uint16_t t = data[2];
    t |= (data[3] << 8);
    return t;
}

uint16_t get_humidity(){
    read_data();

    uint16_t h = data[0];
    h |= (data[1] << 8);
    return h;
}


void application(){
    // Get sensor readings
    temp = (get_temperature() >> 1);
    temp += (get_temperature() >> 1);

    humidity = (get_humidity() >> 1);
    humidity += (get_humidity() >> 1);

    uint32_t output = (temp << 32) | humidity;
    SECURE_record_output_data(output);
}
#endif // TEMP

#if APP_SEL == ULT

#define MAX_DURATION    1000
extern __IO uint32_t uwTick;

void delay(uint32_t us){
    SECURE_new_log_entry();
    int i = uwTick;
    while(i+us>=uwTick);
}

long pulseIn(void){
    SECURE_new_log_entry();
    unsigned long duration = 0;
    int i = 0;

    while(i <= MAX_DURATION){
        duration += (GPIOA->IDR & GPIO_PIN_8) >> 8;
        i++;
    } 

    return duration;
 }

long getUltrasonicReading(void){
    //SECURE_new_log_entry();
    // Set as output and Set signal low for 2us
    GPIOA->BSRR = (uint32_t)GPIO_PIN_8;
    
    delay(2);

    // Set signal high for 5 us
    GPIOA->BRR = (uint32_t)GPIO_PIN_8;

    delay(5);

    // Set signal low
    GPIOA->BSRR = (uint32_t)GPIO_PIN_8;

    // Set as input and read for duration
    unsigned long duration = pulseIn();

    return duration;
}

#define MAX_READINGS    5
void application(){
    //SECURE_new_log_entry();
    uint32_t ult_vec = 0;
    for(int i=0; i<MAX_READINGS; i++){
        ult_vec += getUltrasonicReading()/MAX_READINGS;
    }
    
    SECURE_record_output_data(ult_vec);

}
#endif

#if APP_SEL == DIJKSTRA

#include <stdint.h>
#define INF 0xffffffff
#define MAX_NODES 100

uint8_t dist[MAX_NODES]; // array to store the shortest distances
uint8_t visited[MAX_NODES]; // array to track visited nodes
uint8_t graph[MAX_NODES][MAX_NODES]; // adjacency matrix for the graph
uint8_t n = MAX_NODES * MAX_NODES; // number of nodes in the graph

void my_memset(uint8_t * buff, uint8_t value, uint8_t size){
    unsigned int i = 0;
    do{
        buff[i] = INF;
        i++;
    } while(i < size);
    SECURE_new_log_entry();
}

uint8_t dijkstra(uint8_t start, uint8_t end) {
    my_memset(dist, INF, MAX_NODES); // set all distances to infinity
    my_memset(visited, 0, MAX_NODES); // mark all nodes as unvisited

    dist[start] = 0; // set the distance to the starting node to 0
    
    for (unsigned int i = 0; i < n; i++) {
        uint8_t u = -1;
        for (uint8_t j = 0; j < n; j++) {
            if (!visited[j] && (u == -1 || dist[j] < dist[u])) {
                u = j;
            }
        }
        if (dist[u] == INF) break; // all remaining nodes are inaccessible

        visited[u] = 1;

        for (unsigned int v = 0; v < n; v++) {
            if (graph[u][v] != INF) {
                uint8_t new_dist = dist[u] + graph[u][v];
                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                }
            }
        }
    }

    return dist[end];
}

void application(){
    // Setup arbitrary adjacency matrix
    unsigned int j = 0;
    unsigned int i;
    do{
        i = 0;
        do{
            graph[i][j] = i%4;
            i++;
        } while(i < MAX_NODES);
        j++;
    } while (j < MAX_NODES);

    uint8_t start = 2;
    uint8_t end = 5;

    uint8_t shortest = dijkstra(start, end);
    SECURE_record_output_data(shortest);
}
#endif

#if APP_SEL == DENSITY

void application(){

    volatile uint8_t run = 1;

    do{
        asm volatile("mov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            "\tmov r0, r0\n"
            );
        run++;
    }while(1);
}

#endif

#if APP_SEL == ATTACK
// Password
char pass[4] = {'a', 'b', 'c', 'd'};

// Valid -- correct password
// char user_input[5] = {'a', 'b', 'c', 'd', '\r'};
// Valid -- incorrect password
// char user_input[5] = {'a', 'c', 'c', 'd', '\r'};
//
// Buffer overflow -- jump to 'grant access' when password is wrong
// overwrites: return address to 0x8040450, stack pointer to 0x2003ffe0
char user_input[21] = {0x01, 0x02, 0x03, 0x04, 0x11, 0x12, 0x13, 0x14, 0x21, 0x22, 0x23, 0x24, 0xe0, 0xff, 0x03, 0x20, 0x51, 0x04, 0x04, 0x08, '\r'};

// Output data
long ult_readings[TOTAL_READINGS] = {0,0,0,0,0,0};

uint16_t sensor = 0xa5;

//---------- Check "password" code (vulnerable to buffer-overflow) ----------//
char waitForPassword(){
    //SECURE_new_log_entry();
    char entry[4] = {0,0,0,0};

    read_data(entry);

    char total = 0;
    unsigned int i = 0;
    while(i < 4) 
    //do 
    {
        total |= (pass[i] ^ entry[i]);
        i++;
    } //while(i < 4);

    return total;
}

// uint32_t output_data = 0;
// uint8_t * output = &output_data;
void read_data(char * entry){
    // simulate uart receive
    //SECURE_new_log_entry();
    int i = 0;
    while(user_input[i] != cr)//do
    {
        // save read value
        // output[0] = entry[i];
        // SECURE_record_output_data(output_data);
        // output_data = (output_data << 8);
        entry[i] = user_input[i];
        i++;
    } //while(user_input[i] != cr);
}
//---------- Gather ultrasonic sensor readings ----------//
void delay(unsigned int us){
    //SECURE_new_log_entry();
    int i = 0;
    while(i<=us)//do
    {
        i++;
    } //while(i<=us);
}

long pulseIn(void){
    //SECURE_new_log_entry();
    unsigned long duration = 0;
    int i = 0;
    while(i <= MAX_DURATION)//do
    {
        duration += (P2IN & ULT_PIN);
        i++;
    } //while(i <= MAX_DURATION);

    return duration;
 }

long getUltrasonicReading(void){
    //SECURE_new_log_entry();

    // Set as output
    P2DIR |= ULT_PIN;

    //Set signal low for 2us
    P2OUT &= ~ULT_PIN;
    delay(2);

    // Set signal high for 5 us
    P2OUT |= ULT_PIN;
    delay(5);

    // Set signal low
    P2OUT &= ~ULT_PIN;
    
    // Set as input
    P2DIR &= ~ULT_PIN;

    unsigned long duration = pulseIn();
    
    return duration;
}

// --------------------- Main ------------------//
uint8_t flag;
void application(void)
{
    //SECURE_new_log_entry();
    flag = 0xff;
    P2DIR = 0x00;
    
    char total = waitForPassword();
    // uint32_t app_data = (0xaaaaaa << 8) | total;
    // SECURE_record_output_data(app_data);
    if(total != 0){ // Deny access
        flag = 0x00;
    }
    else { // Grant access
        flag = 0x01;
        // SECURE_record_output_data(flag);
        unsigned char i = 0;
        while(i < TOTAL_READINGS){
            ult_readings[i] = getUltrasonicReading();
            i++;
        } //while(i < TOTAL_READINGS);
    }
}
#endif

#if APP_SEL == ATTACK2

// --------------------- Main ------------------//
#define MAX_READINGS        83
#define MAX_DURATION        1000
#define cmd_ult             'u'
#define cmd_temp_hum        't'
#define cmd_all             'a'

// Ultrasonic only
// char input[4] = {'u', 0, 16, ':'};
// Temperature and humidity
// uint8_t input[7] = {'t', 0, 2, 'h', 0, 2, ':'};
// All
//char input[13] = {'a', 0, 1, 'u', 0, 16, 't', 0, 2, 'h', 0, 2, ':'};
// Attack -- an "all" request modified to run only ultrasonic 
// overwrites: return address to 0x804077e, stack pointer to 0x2003ffe8                                                 8040812 
// char input[25] = {'a', 0, 1, 'u', 0, 16, 't', 0, 2, 'h', 0, 2, 'a','a','a','a', 0xe8, 0xff, 0x03, 0x20, 0x13, 0x08, 0x04, 0x08, ':'};
//                                                                                                                                                             8040694
char input[33] = {'b','b','b','b','b','b','b','b','b','b','b','b','b','b','b','b','b','b','b','b','b','b','b','b', 0xe8, 0xff, 0x03, 0x20, 0x95, 0x06, 0x04, 0x08, ':'};

uint32_t data[4] = {0,0,0,0};
uint8_t temp_data[5] = {0,0,0,0,0};
uint8_t valid_reading = 0;

int ult_runs;
int temp_runs;
int hum_runs;
int seq_runs;
int runs;

extern __IO uint32_t uwTick;

void delay(unsigned int us){
    SECURE_new_log_entry();
    int start = uwTick;
    // while(uwTick-start<=us);
    for(int current=start; uwTick-current<us; current = uwTick);
}

void read_data(void){
    SECURE_new_log_entry();
    uint8_t counter = 0;
    uint16_t j = 0, i;

    // pull signal high & delay
    GPIOA->BRR = (uint32_t)GPIO_PIN_8;
    delay(250);

    /// pull signal low for 20us
    GPIOA->BSRR = (uint32_t)GPIO_PIN_8;
    delay(20);

    // pull signal high for 40us
    GPIOA->BRR = (uint32_t)GPIO_PIN_8;
    delay(40);

    //read timings
    for(i=0; i<MAX_READINGS; i++){

        counter += (GPIOA->IDR & GPIO_PIN_8) >> 8;

        // ignore first 3 transitions
        if ((i >= 4) && ( (i & 0x01) == 0x00)) {

            // shove each bit into the storage bytes
            temp_data[j >> 3] <<= 1;
            if (counter > 6){

                data[j >> 3] |= 1;
            }

            j++;
        }

    }
    // check we read 40 bits and that the checksum matches
    if ((j >= 40) && (temp_data[4] == ((temp_data[0] + temp_data[1] + temp_data[2] + temp_data[3]) & 0xFF)) ) {

        valid_reading = 1;
    } else {

        valid_reading = 0;
    }
}

uint16_t get_temperature(void){
    SECURE_new_log_entry();
    read_data();

    uint16_t t = temp_data[2];
    t |= (temp_data[3] << 8);
    return t;
}

uint16_t get_humidity(void){
    SECURE_new_log_entry();
    read_data();

    uint16_t h = temp_data[0];
    h |= (temp_data[1] << 8);
    return h;
}

long pulseIn(void){
    SECURE_new_log_entry();
    unsigned long duration = 0;
    int i = 0;

    do{
        duration += (GPIOA->IDR & GPIO_PIN_8) >> 8;
        i++;
    } while(i <= MAX_DURATION);

    return duration;
 }

long getUltrasonicReading(){
    SECURE_new_log_entry();
    // Set as output and Set signal low for 2us
    GPIOA->BSRR = (uint32_t)GPIO_PIN_8;

    delay(2);

    // Set signal high for 5 us
    GPIOA->BRR = (uint32_t)GPIO_PIN_8;

    delay(5);

    // Set signal low
    GPIOA->BSRR = (uint32_t)GPIO_PIN_8;

    // Set as input and read for duration
    unsigned long duration = pulseIn();

    return duration;
}

long run_ultrasonic(int runs){
    SECURE_new_log_entry();
    int i;
    long total = 0;
    for(i=0; i<runs; i++){
        total += getUltrasonicReading()/runs;
    }
    return total;
}

long run_temperature(int runs){
    SECURE_new_log_entry();
    int i;
    long total = 0;
    for(i=0; i<runs; i++){
        total += get_temperature()/runs;
    }
    return total;
}

long run_humidity(int runs){
    SECURE_new_log_entry();
    int i;
    long total = 0;
    for(i=0; i<runs; i++){
        total += get_humidity()/runs;
    }
    return total;
}

uint32_t send_data; ///2003ffc0
void read_command(char * msg, char * input){
    while(*input != ':'){
        *msg = *input;
        SECURE_record_output_data(*input);
        SECURE_record_output_data(*msg);
        msg++;
        input++;
    }
}

char cmd;
void process_command(){
    char msg[16] = {'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a'};
    read_command(msg, input);

    switch(msg[0]){
        case cmd_ult:
            ult_runs = (msg[1] << 8) | msg[2];
            break;
        case cmd_temp_hum:
            temp_runs = (msg[1] << 8) | msg[2];
            hum_runs = (msg[4] << 8) | msg[5];
            break;
        case cmd_all:
            seq_runs  = (msg[1]  << 8) | msg[2];
            ult_runs  = (msg[4]  << 8) | msg[5];
            temp_runs = (msg[7]  << 8) | msg[8];
            hum_runs  = (msg[10] << 8) | msg[11];
        default:
            break;
    }
    SECURE_record_output_data(msg[0]);
    // return msg[0];
    cmd = msg[0];
}

int (*sensor_action)(char);

int record_output_data(char data){
    SECURE_new_log_entry();
    return data^0xff;
}

void application(void)
{

    process_command();

    switch(cmd){
        case cmd_ult:
            runs = record_output_data(cmd_ult);
            // data[0] = run_ultrasonic(ult_runs);
            sensor_action = &run_ultrasonic;
            runs = ult_runs;
            break;
        case cmd_temp_hum:
            runs = record_output_data(cmd_temp_hum);
            // data[1] = run_temperature(temp_runs);
            // data[2] = run_humidity(hum_runs);
            sensor_action = &run_temperature;
            runs = temp_runs;
            break;
        case cmd_all:
            runs = record_output_data(cmd_all);
            // data[0] = run_ultrasonic(ult_runs);
            // data[1] = run_temperature(temp_runs);
            // data[2] = run_humidity(hum_runs);
            sensor_action = &run_humidity;
            runs = hum_runs;
            break;
        default:
            break;
    }

    sensor_action(runs);
}
#endif
