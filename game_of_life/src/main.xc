// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include <assert.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 512                                      //image height
#define  IMWD 512                                      //image width
#define  numberOfWorkers 4
#define  numberOfRowsInSlice (IMHT / numberOfWorkers)
#define  maxTicks 4294967295                            //size of unsigned int
#define  maxTicksMS 42950                               //size of unsigned int to ms precision

typedef unsigned char uchar;                            //using uchar as shorthand

char infname[] = "game_of_life/512x512.pgm";            //put your input image path here
char outfname[] = "game_of_life/testout.pgm";           //put your output image path here

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
const uchar mask[] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

on tile[0] : in port buttons = XS1_PORT_4E;     //port to access xCore-200 buttons
on tile[0] : out port leds = XS1_PORT_4F;       //port to access xCore-200 LEDs
on tile[0] : port p_scl = XS1_PORT_1E;          //interface ports to orientation
on tile[0] : port p_sda = XS1_PORT_1F;

#define FXOS8700EQ_I2C_ADDR 0x1E                //register addresses for orientation
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

////DataInStream for .pgm files
//void DataInStream(char infname[], chanend c_out)
//{
//  int res;
//  uchar line[ IMWD ];
//  printf( "DataInStream: Start...\n" );
//
//  //Open PGM file
//  res = _openinpgm( infname, IMWD, IMHT );
//  if( res ) {
//    printf( "DataInStream: Error openening %s\n.", infname );
//    return;
//  }
//
//  //Read image line-by-line and send byte by byte to channel c_out
//  for( int y = 0; y < IMHT; y++ ) {
//    _readinline( line, IMWD );
//    for( int x = 0; x < IMWD; x++ ) {
//      c_out <: line[ x ];
//      //printf( "-%4.1d ", line[ x ] ); //show image values
//    }
//    //printf( "\n" );
//  }
//
//  //Close PGM image file
//  _closeinpgm();
//  printf( "DataInStream: Done...\n" );
//  return;
//}

//DataInStream for randomly generated images of variable size
void DataInStream(char infname[], chanend c_out)
{
  printf( "DataInStream: Start...\n" );

  uchar random;
  for( int y = 0; y < IMHT; y++ ) {
      for( int x = 0; x < IMWD/8; x++ ) {
          random = rand() % 255 ;
          c_out <: random;
      }
  }
  printf( "DataInStream: Done...\n" );
  return;
}



//////////////////////////////// Our functions///////////////////////////////////////////


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

//Dont send 2 request before 1 please.
void longTimer(chanend fromDistributor){ //
    timer t;
    unsigned int startTime = 0;
    unsigned int ticker = 0;
    unsigned int oldTicker = 0;
    unsigned int loopCount = 0;
    unsigned int pausedTime = 0;
    unsigned int startCumulativeTime = 0;
    unsigned int endCumulativeTime = 0;

    fromDistributor :> int start; //Distributor starts the timer.
    t :> ticker;
    startTime = (ticker / 100000); //storing time in ms

    while (1){
        select {
            case fromDistributor :> int request: //Pinged from Distributer
                switch(request) {
                case 1: //start of pause
                    t :> ticker;
                    startCumulativeTime = ((ticker / 100000) + loopCount * (maxTicksMS)) - pausedTime - startTime;
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
                if (ticker < oldTicker){ //tracks number of times timer t resets to 0
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

//extracts bit value at 'n' position of register 'reg'.
uchar extractBit (uchar reg, int n) {
    if ((reg & mask[n]) != 0) return 255;
    else return 0;
}

//Returns dead or alive for a coordinate in unpacked map.
uchar extractCoordinate(uchar map[rows][IMWD/8], int x, int y, unsigned int rows) {
    int xRegister = x / 8;
    int xPos = x % 8;

    uchar cell = extractBit(map[y][xRegister], xPos);
    return cell;
}

////readInMap for .pgm files
//void readInMap(chanend c_in, uchar image[IMHT][IMWD/8]){
//    uchar val;
//    //Reads in pgm, cell by cell.
//    for(int y = 0; y < IMHT; y++ ) {
//        for (int x = 0; x < (IMWD / 8); x++){
//            uchar target = 0;
//            for (int position = 1; position < 9; position++){ //bitpacks the map as it is read in.
//                c_in :> val;
//                if (val == 255){
//                    target |= (0x01 << (8 - position));
//                }
//            }
//            image[y][x] = target;
//        }
//    }
//}

//readInMap for randomly generated images of variable size
void readInMap(chanend c_in, uchar image[IMHT][IMWD/8]){
    //Reads in pgm, cell by cell.
    for(int y = 0; y < IMHT; y++ ) {
        for (int x = 0; x < (IMWD / 8); x++){
            c_in :> image[y][x];
        }
    }
}

//Exports current game state to testout.pgm
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

uchar calculateCell(uchar map[rows][IMWD/8], int x, int y, unsigned int rows) { //'x' and 'y' refer to abstract map coordinates.
    int alive = extractCoordinate(map, x, y, rows);

    uchar surroundingCells[8];
    //topleft, topmiddle, topright, left, right, bottomleft, bottommiddle, bottomright.
    //Finds surrounding cells based on abstract coordinates (as if it was not bitpacked)

    surroundingCells[0] = extractCoordinate(map, ((x - 1 + IMWD) % IMWD), (y - 1), rows);
    surroundingCells[1] = extractCoordinate(map, ((x + IMWD) % IMWD), (y - 1), rows);
    surroundingCells[2] = extractCoordinate(map, ((x + 1 + IMWD) % IMWD), (y - 1), rows);
    surroundingCells[3] = extractCoordinate(map, ((x - 1 + IMWD) % IMWD), (y), rows);
    surroundingCells[4] = extractCoordinate(map, ((x + 1 + IMWD) % IMWD), (y), rows);
    surroundingCells[5] = extractCoordinate(map, ((x - 1 + IMWD) % IMWD), (y + 1), rows);
    surroundingCells[6] = extractCoordinate(map, ((x + IMWD) % IMWD), (y + 1), rows);
    surroundingCells[7] = extractCoordinate(map, ((x + 1 + IMWD) % IMWD), (y + 1), rows);

    uchar calculatedCell = calculateCellHelper(surroundingCells, alive);
    return calculatedCell;
}

//-------This version was for before worker threads were implemented-----------
//void calculateMap(uchar mapIn[IMHT][IMWD/8], uchar mapOut[IMHT][IMWD/8]){
//        uchar calculatedRegister;
//        for(int y = 0; y < IMHT; y++ ) {
//            for (int x = 0; x < IMWD/8; x++ ){  // for loop for 8 cells
//                calculatedRegister = 0;
//                for (int bit = 1; bit < 9; bit ++){
//                    if (calculateCell(mapIn, (x*8+bit-1), y) == 255){
//                        calculatedRegister |= (0x01 << (8 - bit));
//                    }
//                }
//                mapOut[y][x] = calculatedRegister;
//            }
//        }
//}

//Calculates current number of live cells
int numLiveCells(uchar map[rows][IMWD/8], unsigned int rows){
    int liveCells = 0;
    uchar alive = 0;
    for (int y = 0; y < IMHT; y++){
        for (int x = 0; x < IMWD; x++){
            alive = extractCoordinate(map, x, y, rows);
            if (alive == 255) liveCells ++;
        }
    }
    return liveCells;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Concurrent Functions
//
/////////////////////////////////////////////////////////////////////////////////////////

//Sends bitpacked map
//numRows doesn't include ghost rows
//startRow is the first relevant row for the slice(not ghost row)
void sendImageSlice(chanend toWorker, uchar image[IMHT][IMWD/8], int numRows, int startRow){
    int topGhostRow = (startRow - 1 + IMHT) % IMHT;
    int bottomGhostRow = (startRow + numRows) % IMHT;

    //Send top ghost row
    for (int x = 0; x < IMWD/8; x++){
        toWorker <: image[topGhostRow][x];
    }

    //Send target rows
    for (int y = startRow; y < (startRow + numRows); y++){
        for (int x = 0; x < IMWD/8; x++){
            toWorker <: image[y][x];
        }
    }

    //Send bottom ghost row
    for (int x = 0; x < IMWD/8; x++){
        toWorker <: image[bottomGhostRow][x];
    }
}

//Receives calculated bytes from workers and puts them into the correct spot in map.
void recompileMap(chanend toWorker, uchar image[IMHT][IMWD/8], int workerNumber){

    for (int y = (workerNumber*(IMHT/numberOfWorkers)); y < (IMHT*(workerNumber+1))/numberOfWorkers; y ++){
        for (int x = 0; x < IMWD / 8; x ++){
            toWorker :> image[y][x];
        }
    }
}

//Slices map and distributes to worker threads for processing
void calculateMap(chanend toWorker[], uchar image[IMHT][IMWD/8]){

    //Sends all slices
    for (int i = 0; i < numberOfWorkers; i ++){
        sendImageSlice(toWorker[i], image, numberOfRowsInSlice, i * numberOfRowsInSlice);
    }

    //Receives all slices and recompiles into image
    for (int i = 0; i < numberOfWorkers; i++){
        recompileMap(toWorker[i], image, i);
    }
}

//A thread that progresses an image slice
void worker(chanend fromDistributor, int rowsInSlice){
    timer t;
    int startTimer;
    int endTimer;


    uchar imageSlice[numberOfRowsInSlice+3][IMWD/8];

    //Worker runs continuously.
    while(1){
        //Read in bitpacked slice
        for (int y = 1; y < (rowsInSlice + 3); y++){
            for (int x = 0; x < IMWD/8; x ++){
                t :> startTimer;
                fromDistributor :> imageSlice[y][x];
                t :> endTimer;
                printf("%u\n", endTimer - startTimer);
            }
        }

        //Loops over relevant bytes, extracts cells, calculates new cells, bitpacks them and exports to distributor
        uchar calculatedRegister;
        for (int y = 2; y < rowsInSlice + 2; y ++){
            for (int x = 0; x < IMWD/8; x ++){
                calculatedRegister = 0;
                for (int bit = 1; bit < 9; bit ++){
                    if (calculateCell(imageSlice, (x*8+(bit-1)), y, (rowsInSlice + 3)) == 255){
                        calculatedRegister |= (0x01 << (8 - bit));
                    }
                }
                imageSlice[y - 2][x] = calculatedRegister;
            }

        }

        //Exports relevant cells to the distributor
        for (int y = 0; y < rowsInSlice; y ++){
            for (int x = 0; x < IMWD/8; x ++){
                fromDistributor <: imageSlice[y][x];
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////

void distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend fromButtons,
        chanend toLED, chanend fromTimer, chanend toWorker[numberOfWorkers])
{
  uchar image[IMHT][IMWD/8];
  int round = 0;
  int orientation = 0;
  int numberLiveCells = 0;
  unsigned int currentTime;

  //Starting up and wait for SW1 press
  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for 'SW1' button press...\n" );

  int patternLED = 0;
  int value = 15;            //15 is 'no button pressed'
  while(value != 14) {       //Wait for 'SW1' from buttonListener to start processing
      fromButtons :> value;
  }
  value = 15;

  printf( "Reading in map.\n" );
  patternLED = 4;
  toLED <: patternLED; //Turns on green LED

  //Reads in map in bitmap format.
  readInMap(c_in, image);

  patternLED = 0;
  toLED <: patternLED; //Turns off green LED
  printf("Image read out succesfully.\n");

  printf("Processing.\n" );

  fromTimer <: 0; //starts the timer.

  int running = 1;
  while(running){
      //Pauses calculations when the board is tilted.
      fromAcc :> orientation;

//      if (round == 1){
//          printf("readingOutMap\n");
//          readOutMap(image, c_out);
//      }
//      printf("Finished reading out out map.\n");

      //Pause when tilted
      if (orientation == 1){
          fromTimer <: 1;           //pauses the timer.
          fromTimer :> currentTime; //receive ticker time

          patternLED = 8;           //Turns LED red.
          toLED <: patternLED;

          //Reads off stats
          printf("\nProcessing paused.\n");
          printf("%ums elapsed since processing started.\n", currentTime);
          printf("%ums per processing round.\n", currentTime / round );
          printf("%u rounds processed.\n", round);

          numberLiveCells = numLiveCells(image, IMHT);
          printf("There are currently %d live cells.\n\n", numberLiveCells);

          while(orientation){ //Loops to wait for resume signal
              fromAcc :> orientation;
          }
          fromTimer <: 2; //resumes timer.


      }

      //prints out image at specific round
//      if (round == 2){
//          readOutMap(image, c_out);
//      }
      //numberLiveCells = numLiveCells(image, IMHT);
      //printf("There are currently %d live cells.\n\n", numberLiveCells);


      //Calculates map and blinks LED on alternate rounds
      if ((round % 2) == 0){
          patternLED = 0;
          toLED <: patternLED;
          calculateMap(toWorker, image);
      }
      else {
          patternLED = 1;
          toLED <: patternLED;
          calculateMap(toWorker, image);
      }
      //printf( "Processing round: %d completed...\n", round);

      //wait for 'SW2' from buttonListener
      select {
          case fromButtons :> value:
              if (value == 13) {

                  //update LEDs
                  patternLED = 2;
                  toLED <: patternLED;
                  printf("Reading out map.\n");
                  readOutMap(image, c_out);
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
  printf("Distributor no longer running.\n");
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

//void calculateMap(uchar mapIn[IMHT][IMWD/8], uchar mapOut[IMHT][IMWD/8]){

//8x8 test map;
//uchar testArray[8][1] = {{0x02}, {0x01}, {0x38}, {0x31}, {0xa9}, {0x00}, {0x00}, {0x80}};
//uchar firstIteration[8][1] = {{0x01}, {0x10}, {0x28}, {0x81}, {0xE1}, {0x00}, {0x00}, {0x00}};
//uchar secondIteration[8][1] = {{0x00}, {0x10}, {0x10}, {0x31}, {0x41}, {0xc0}, {0x00}, {0x00}};
//
////Output arrays.
//uchar firstIterationOutput[8][1];
//uchar secondIterationOutput[8][1];
//
//void testCalculateMap(uchar testArray[8][1], uchar firstIteration[8][1],
//                      uchar secondIteration[8][1],
//                      uchar firstIterationOutput[8][1],
//                      uchar secondIterationOutput[8][1]) {
//
//    calculateMap(testArray, firstIterationOutput);
//    printf("firstIterationOutput:\n");
//    for (int y = 0; y < 8; y++) {
//        printf("%x, ", firstIterationOutput[y][0]);
//    }
//    printf("\n");
//
//    for (int y = 0; y < 8; y++) {
//        assert(firstIterationOutput[y][0] == firstIteration[y][0]);
//    }
//
//    calculateMap(firstIteration, secondIterationOutput);
//    printf("secondIterationOutput:\n");
//    for (int y = 0; y < 8; y++) {
//        printf("%x, ", secondIterationOutput[y][0]);
//    }
//    printf("\n");
//
//    for (int y = 0; y < 8; y++) {
//        assert(secondIterationOutput[y][0] == secondIteration[y][0]);
//    }
//
//    printf("Test: calculateMap passed.\n");
//}


/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////

int main(void) {

i2c_master_if i2c[1];               //interface to orientation

chan c_inIO, c_outIO, c_control, c_buttons, c_LEDs, c_timer;    //extend your channel definitions here
chan c_worker[numberOfWorkers];

par {
    on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    on tile[0] : orientation(i2c[0],c_control);        //client thread reading orientation data
    on tile[0] : DataInStream(infname, c_inIO);          //thread to read in a PGM image
    on tile[0] : DataOutStream(outfname, c_outIO);       //thread to write out a PGM image
    on tile[0] : buttonListener(buttons, c_buttons); //thread to listen for button presses
    on tile[0] : showLEDs(leds, c_LEDs);

    on tile[0] : longTimer(c_timer);
    on tile[0] : distributor(c_inIO, c_outIO, c_control, c_buttons, c_LEDs, c_timer, c_worker);//thread to coordinate work on image
    par (int i = 0; i < numberOfWorkers; i++){
        on tile[1] : worker(c_worker[i], numberOfRowsInSlice);
    }

    //testCalculateMap(testArray, firstIteration, secondIteration, firstIterationOutput, secondIterationOutput);
  }

return 0;
}
