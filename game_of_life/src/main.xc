// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include <assert.h>
//#include "xcore_c.h"
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 64                   //image height
#define  IMWD 64                   //image width
#define  maxTicks 4294967295       //size of int
#define  maxTicksMS 42950          //size of int to ms precision

typedef unsigned char uchar;      //using uchar as shorthand

//definitions for bit-packing
#define BIT8 0x01
#define BIT7 0x02
#define BIT6 0x04
#define BIT5 0x08
#define BIT4 0x10
#define BIT3 0x20
#define BIT2 0x40
#define BIT1 0x80
const uchar bits[8] = {BIT1, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7, BIT8};

on tile[0] : in port buttons = XS1_PORT_4E; //port to access xCore-200 buttons
on tile[0] : out port leds = XS1_PORT_4F;   //port to access xCore-200 LEDs

struct coordinates {
    int x;
    int y;
};
typedef struct coordinates coordinates;

port p_scl = XS1_PORT_1E;         //interface ports to orientation
port p_sda = XS1_PORT_1F;

#define FXOS8700EQ_I2C_ADDR 0x1E  //register addresses for orientation
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1
#define FXOS8700EQ_OUT_X_LSB 0x2
#define FXOS8700EQ_OUT_Y_MSB 0x3
#define FXOS8700EQ_OUT_Y_LSB 0x4
#define FXOS8700EQ_OUT_Z_MSB 0x5
#define FXOS8700EQ_OUT_Z_LSB 0x6

/////////////////////////////////////////////////////////////////////////////////////////
//
// Read Image from PGM file from path infname[] to channel c_out
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataInStream(char infname[], chanend c_out)
{
  int res;
  uchar line[ IMWD ];
  printf( "DataInStream: Start...\n" );

  //Open PGM file
  res = _openinpgm( infname, IMWD, IMHT );
  if( res ) {
    printf( "DataInStream: Error openening %s\n.", infname );
    return;
  }

  //Read image line-by-line and send byte by byte to channel c_out
  for( int y = 0; y < IMHT; y++ ) {
    _readinline( line, IMWD );
    for( int x = 0; x < IMWD; x++ ) {
      c_out <: line[ x ];
      //printf( "-%4.1d ", line[ x ] ); //show image values
    }
    //printf( "\n" );
  }

  //Close PGM image file
  _closeinpgm();
  printf( "DataInStream: Done...\n" );
  return;
}
/////////////////////////////////////////////////////////////////////////////////////////
//
// Our functions.
//
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
//
// I/O Functions
//
/////////////////////////////////////////////////////////////////////////////////////////

//READ BUTTONS and send button pattern
void buttonListener(in port b, chanend toDistributor) { //13 = 'SW2', 14 = 'SW1'
  int r;
  while (1) {
    b when pinseq(15)  :> r;    // check that no button is pressed
    b when pinsneq(15) :> r;    // check if some buttons are pressed
    if ((r==13) || (r==14))     // if either button is pressed
    toDistributor <: r;         // send button pattern to 'distributor'
  }
}

//DISPLAYS an LED pattern
int showLEDs(out port p, chanend fromDistributor) {
  int pattern; //1st bit...separate green LED
               //2nd bit...blue LED (2)
               //3rd bit...green LED (4)
               //4th bit...red LED (8)
  while (1) {
    fromDistributor :> pattern;   //receive new pattern from visualiser
    p <: pattern;                //send pattern to LED port
  }
  return 0;
}

//(smallTimer * 1000) / XS1_TIMER_MHZ)
//Dont send 2 request before 1 please.
void longTimer(chanend fromDistributor){ //
    timer t;
    unsigned int startTime = 0;
    unsigned int ticker = 0;
    unsigned int oldTicker = 0;
    unsigned int loopCount = 0;
    //unsigned long long pausedTime, startCumulativeTime, endCumulativeTime = 0;
    unsigned int pausedTime, startCumulativeTime, endCumulativeTime = 0;


    //Start the clock
    fromDistributor :> int start; //Starts the timer.
    t :> ticker;
    printf("Ticker initialized at %d\n", ticker);
    //fromDistributor <: (ticker / 100000);
    startTime = (ticker / 100000);
    printf("start time initialized at %d\n", startTime);


    while (1){
        select {
            case fromDistributor :> int request: //Pinged from Distributer
                switch(request) {
                case 1: //start of pause
                    t :> ticker;
                    startCumulativeTime = ((ticker / 100000) + loopCount * (maxTicksMS)) - pausedTime - startTime;
                    printf("cumulative time from long timer is %u\n", startCumulativeTime);
                    printf("ticker from long timer is %u\n", ticker);
                    printf("loopCount * (maxTicksMS) from long timer is %u\n", loopCount * (maxTicksMS));
                    printf("pausedTime from long timer is %u\n", pausedTime);
                    fromDistributor <: startCumulativeTime;
                    break;
                case 2: //end of pause
                    t :> ticker;
                    endCumulativeTime = ((ticker / 100000) + loopCount * (maxTicksMS)) - pausedTime - startTime;
                    pausedTime += (endCumulativeTime - startCumulativeTime);
                    break;
                default:
                    printf("You passed an invalid request to the longTimer.\n");
                    break;
                }
                break;
            default :
                t :> ticker;
                if (ticker < oldTicker){
                    loopCount ++;
                }
                oldTicker = ticker;
                break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Logic Functions
//
/////////////////////////////////////////////////////////////////////////////////////////

uchar extractBit (uchar reg, int n) {//extracts bit value at 'n' position of register 'reg'.
    static uchar mask[] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
    if ((reg & mask[n]) != 0) return 255;
    else return 0;
}

//Returns dead or alive for a coordinate in unpacked map.
uchar extractCoordinate(uchar map[IMHT][IMWD/8], int x, int y) {
    int xRegister = x / 8;
    int xPos = x % 8;

    uchar cell = extractBit(map[y][xRegister], xPos);
    return cell;
}

void readOutMap(uchar map[IMHT][IMWD/8], chanend c_out) { //Sends map to dataOutStream.
    for(int y = 0; y < IMHT; y++ ) {
        for (int x = 0; x < IMWD/8; x++ ){
            for (int bit = 0; bit < 8; bit ++){
                //c_out <: extractCoordinate(map, ((8*x) + bit), y);
                c_out <: extractBit(map[y][x], bit);
            }
        }
    }
}

uchar calculateCellHelper(uchar grid[8], int alive) {
    int a = alive;
    int count = 0;

    for (int i = 0; i < 8; i++) { //Counts number of alive neighbour cells.
        if (grid[i] == 255) count ++;
    }

    if (a == 255) { //Logic for if cell is alive.
        if (count < 2) a = 0;
        else if (count == 3 || count == 2) a = a; //Do nothing.
        else if (count > 3 && count < 9) a = 0;
        else {
            printf("Variable 'count' in calculateCell out of bounds.\n");
            exit(1);
        };
    }
    else { //Logic for dead cell.
        if (count == 3) a = 255;
    }

    //Returns dead or alive (cell state) in uchar format.
    return (uchar) a;
}

uchar calculateCell(uchar map[IMHT][IMWD/8], int x, int y) { //'x' and 'y' refer to abstract map coordinates.
    int alive = extractCoordinate(map, x, y);

    uchar surroundingCells[8];
    //topleft, topmiddle, topright, left, right, bottomleft, bottommiddle, bottomright.
    surroundingCells[0] = extractCoordinate(map, ((x - 1 + IMWD) % IMWD), ((y - 1 + IMHT) % IMHT));
    surroundingCells[1] = extractCoordinate(map, ((x + IMWD) % IMWD), ((y - 1 + IMHT) % IMHT));
    surroundingCells[2] = extractCoordinate(map, ((x + 1 + IMWD) % IMWD), ((y - 1 + IMHT) % IMHT));
    surroundingCells[3] = extractCoordinate(map, ((x - 1 + IMWD) % IMWD), ((y + IMHT) % IMHT));
    surroundingCells[4] = extractCoordinate(map, ((x + 1 + IMWD) % IMWD), ((y + IMHT) % IMHT));
    surroundingCells[5] = extractCoordinate(map, ((x - 1 + IMWD) % IMWD), ((y + 1 + IMHT) % IMHT));
    surroundingCells[6] = extractCoordinate(map, ((x + IMWD) % IMWD), ((y + 1 + IMHT) % IMHT));
    surroundingCells[7] = extractCoordinate(map, ((x + 1 + IMWD) % IMWD), ((y + 1 + IMHT) % IMHT));

    uchar calculatedCell = calculateCellHelper(surroundingCells, alive);
    return calculatedCell;
}

void calculateMap(uchar mapA[IMHT][IMWD/8], uchar mapB[IMHT][IMWD/8]){
        uchar calculatedRegister;
        for(int y = 0; y < IMHT; y++ ) {
            for (int x = 0; x < IMWD/8; x++ ){  // for loop for 8 cells
                calculatedRegister = 0;
                for (int bit = 1; bit < 9; bit ++){
                    if (calculateCell(mapA, (x*8+bit-1), y) == 255){
                        calculatedRegister |= (0x01 << (8 - bit));
                    }
                }
                mapB[y][x] = calculatedRegister;
            }
        }
}

int numLiveCells(uchar map[IMHT][IMWD/8]){

    int liveCells = 0;
    uchar alive = 0;
    for (int y = 0; y < IMHT; y++){
        for (int x = 0; x < IMWD; x++){
            alive = extractCoordinate(map, x, y);
            if (alive == 255) liveCells ++;
        }

    }
    return liveCells;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////

void distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend fromButtons,
        chanend toLED, chanend fromTimer)
{
  uchar val;
  uchar imageA[IMHT][IMWD/8]; //imageA is originally read from and imageB written to...
  uchar imageB[IMHT][IMWD/8]; //...they swap each iteration.
  int round = 0;
  int orientation = 0;
  int numberLiveCells = 0;
//  unsigned int timeElapsed = 0;
//  unsigned long long startTime;
//  unsigned long long currentTime;
  //unsigned int startTime;
  unsigned int currentTime;

  //Starting up and wait for SW1 press
  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for 'SW1' button press...\n" );

  int patternLED = 0;
  int value = 15;
  while(value != 14) {   //Wait for 'SW1' from buttonListener to start processing
      fromButtons :> value;
  }
  value = 15;

  printf( "Reading in map.\n" );
  patternLED = 4;
  toLED <: patternLED;

  //Reads in pgm, cell by cell.
  for(int y = 0; y < IMHT; y++ ) {
      for (int x = 0; x < (IMWD / 8); x++){
          uchar target = 0;
          for (int position = 1; position < 9; position++){ //bitpacks the map as it is read in.
              c_in :> val;
              if (val == 255){
                  target |= (0x01 << (8 - position));
              }
          }

          imageA[y][x] = target;
      }
  }

  patternLED = 0;
  toLED <: patternLED;
  printf("Image read out succesfully.\n");

  fromTimer <: 0; //starts the timer.
  //fromTimer :> startTime;
  //printf("start time = %d\n", startTime);

  printf("Processing.\n" );

  int running = 1;
  while(running){
      //Pauses calculations when the board is tilted.
      fromAcc :> orientation;
      if (orientation == 1){
          fromTimer <: 1; //pauses the timer.
          fromTimer :> currentTime; //receive ticker time
          printf("\nProcessing paused.\n");
          //!!!!!!!!!!!update this so units are correct!!!!!!!!!!!!!!!
          printf("%ums elapsed since processing started.\n", currentTime);
          printf("%u rounds processed.\n", round - 1);
          if ((round % 2) == 0){
              numberLiveCells = numLiveCells(imageA);
          }
          else {
              numberLiveCells = numLiveCells(imageB);
          }
          printf("There are currently %d live cells.\n\n", numberLiveCells);

          patternLED = 8;
          toLED <: patternLED;
          while(orientation){
              fromAcc :> orientation;
          }
          fromTimer <: 2; //resumes timer.
      }
      //Calculates map and blinks LED on alternate rounds
      if ((round % 2) == 0){
          patternLED = 0;
          toLED <: patternLED;
          calculateMap(imageA, imageB);
      }
      else {
          patternLED = 1;
          toLED <: patternLED;
          calculateMap(imageB, imageA);
      }

      printf( "\nProcessing round: %d completed...\n", round);
      //wait for 'SW2' from buttonListener
      select {
          case fromButtons :> value:
              if (value == 13) {

                  //update LEDs
                  patternLED = 2;
                  toLED <: patternLED;
                  printf("Reading out map.\n");

                  if ((round % 2) == 0){
                      readOutMap(imageA, c_out);
                  }
                  else {
                      readOutMap(imageB, c_out);
                  }
                  patternLED = 0;
                  toLED <: patternLED;
              }
              running = 0;
              break;
          default:
              break;
      }
      round++;
    }//End of while loop.
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(char outfname[], chanend c_in)
{
      int res;
      uchar line[ IMWD ];

      //Open PGM file
      printf( "DataOutStream: Start...\n" );
      res = _openoutpgm( outfname, IMWD, IMHT );
      if( res ) {
        printf( "DataOutStream: Error opening %s\n.", outfname );
        return;
      }

      //Compile each line of the image and write the image line-by-line
      for( int y = 0; y < IMHT; y++ ) {
        for( int x = 0; x < IMWD; x++ ) {
          c_in :> line[ x ];
        }
        _writeoutline( line, IMWD );
        //printf( "DataOutStream: Line written...\n" );
      }

      //Close the PGM image
      _closeoutpgm();
      printf( "DataOutStream: Done...\n" );
      return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read orientation, send first tilt event to channel
//
/////////////////////////////////////////////////////////////////////////////////////////
void orientation( client interface i2c_master_if i2c, chanend toDist) {
  i2c_regop_res_t result;
  char status_data = 0;
//  int tilted = 0;

  // Configure FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }
  
  // Enable FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  //Probe the orientation x-axis forever
  while (1) {

    //check until new orientation data is available
    do {
      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
    } while (!status_data & 0x08);

    //get new x-axis tilt value
    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);

    //send signal to distributor after first tilt
      if (x>30 || x < -30) {
        //tilted = 1 - tilted;
        toDist <: 1;
      }
      else{
        toDist <: 0;
      }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Tests.
//
/////////////////////////////////////////////////////////////////////////////////////////

//void testCalculateCell() {
//    uchar grid0[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//    uchar grid1[8] = {0, 0, 0, 0, 0, 0, 0, 255};
//    uchar grid2[8] = {0, 0, 0, 0, 0, 0, 255, 255};
//    uchar grid3[8] = {0, 0, 0, 0, 0, 255, 255, 255};
//    uchar grid4[8] = {0, 0, 0, 0, 255, 255, 255, 255};
//    uchar grid5[8] = {0, 0, 0, 255, 255, 255, 255, 255};
//    uchar grid6[8] = {0, 0, 255, 255, 255, 255, 255, 255};
//    uchar grid7[8] = {0, 255, 255, 255, 255, 255, 255, 255};
//    uchar grid8[8] = {255, 255, 255, 255, 255, 255, 255, 255};
//
//
//    uchar alive = calculateCell(grid7, 1);
//    if (alive == 0) printf("testCalculateCell() passed.\n");
//    else {
//        printf("testCalculateCell() failed.\n");
//    }
//
//}

//uchar calculateCell(uchar *map, int x, int y) { //'x' and 'y' refer to middle cell.

//void DataInStream(char infname[], chanend c_out)
//uchar* aliveSurroundingCells(uchar *map, int x, int y)
//uchar* readInMap(chanend c_in){
//void testSurroundingCells(){
//
//    uchar map[IMHT][IMWD] = malloc(IMHT*IMWD);
//    par{
//        DataInStream("game_of_life/test.pgm", c_inIO);
//        map = readInMap(c_inIO);
//    }
//        uchar grid[] = malloc(8);
//        grid = surroundingCells(map, 1, 1);
//
//}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////

int main(void) {

i2c_master_if i2c[1];               //interface to orientation

char infname[] = "game_of_life/64x64.pgm";     //put your input image path here
char outfname[] = "game_of_life/testout.pgm"; //put your output image path here
chan c_inIO, c_outIO, c_control, c_buttons, c_LEDs, c_timer;    //extend your channel definitions here

par {
    i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    orientation(i2c[0],c_control);        //client thread reading orientation data
    DataInStream(infname, c_inIO);          //thread to read in a PGM image
    DataOutStream(outfname, c_outIO);       //thread to write out a PGM image
    distributor(c_inIO, c_outIO, c_control, c_buttons, c_LEDs, c_timer);//thread to coordinate work on image
    buttonListener(buttons, c_buttons); //thread to listen for button presses
    showLEDs(leds, c_LEDs);
    longTimer(c_timer);
  }


//    //Tests
//    chan c_inIO;
//
//    testCalculateCell();
//
return 0;
}
