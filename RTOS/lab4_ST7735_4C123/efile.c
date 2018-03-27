// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk 
// Jonathan W. Valvano 3/9/17

#include <string.h>
#include "edisk.h"
#include "efile.h"
#include "UART.h"
#include <stdio.h>

#define SUCCESS 0
#define FAIL 1
                             
#define MAXBLOCKNUM 4096
#define DIRBLOCKNUM 2
#define FATBLOCKNUM 14
#define BLOCKSIZE 512

uint8_t FAT[FATBLOCKNUM*BLOCKSIZE];

struct dir{
  char filename[8];
  uint16_t startBlock;
  uint16_t usedBytesNum;
};
typedef struct dir DirType;
DirType directory[DIRBLOCKNUM*42];
uint8_t filebuffer[BLOCKSIZE];
int flagRead;
int writePt;
int filePt = 0;
int readPt;
int readIndex;

//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
int eFile_Init(void){ // initialize file system
  eDisk_Init(0);
  
  uint8_t *bufPt;
  for(int i=0; i<DIRBLOCKNUM; i++)
  {
    if(eDisk_ReadBlock(filebuffer, i)) return FAIL;
    bufPt = filebuffer;
    for(int j=0; j<42; j++)
    {
      for(int k=0; k<8; k++)
      {
        directory[i*42+j].filename[k] = bufPt[k];
      }
      directory[i*42+j].startBlock = (bufPt[8]<<8) | bufPt[9];
      directory[i*42+j].usedBytesNum = (bufPt[10]<<8) | bufPt[11];
      bufPt += 12;
    }
  }
  
  if(eDisk_Read(0, FAT, DIRBLOCKNUM, FATBLOCKNUM)) return FAIL;
  
  return SUCCESS;
}

int saveDIR(void)
{
  uint8_t *bufPt;
  uint8_t res;
  for(int i=0; i<DIRBLOCKNUM; i++)
  {
    bufPt = filebuffer;
    for(int j=0; j<42; j++)
    {
      for(int k=0; k<8; k++)
      {
        bufPt[k] = directory[i*42+j].filename[k];
      }
      bufPt[8] = (directory[i*42+j].startBlock & 0xFF00)>>8;
      bufPt[9] =  directory[i*42+j].startBlock & 0x00FF;
      bufPt[10] = (directory[i*42+j].usedBytesNum & 0xFF00)>>8;
      bufPt[11] = directory[i*42+j].usedBytesNum & 0x00FF;
      bufPt += 12;
    }
    res = eDisk_WriteBlock(filebuffer, i);
    if(res != RES_OK) return FAIL;
  }
  return SUCCESS;
}

int saveFAT(void)
{
  if(eDisk_Write(0, FAT, DIRBLOCKNUM, FATBLOCKNUM)) return FAIL;
  return SUCCESS;
}
//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void){  // erase disk, add format

//  if(result) diskError("eDisk_Init",result);
    
  for(int i=0; i<DIRBLOCKNUM*42; i++)
  {
    directory[i].filename[0] = 0;
    directory[i].startBlock = 0;
    directory[i].usedBytesNum = 0;
  }
  directory[0].filename[0] = '*';
  directory[0].startBlock = DIRBLOCKNUM + FATBLOCKNUM;
  directory[0].usedBytesNum = 0;
  if(saveDIR()) return FAIL;
  
  for(int i=0; i < (DIRBLOCKNUM + FATBLOCKNUM); i++) FAT[i] = 0; //used
  for(int i = (DIRBLOCKNUM + FATBLOCKNUM); i < (MAXBLOCKNUM - 1); i++) FAT[i] = i+1; //used
  FAT[MAXBLOCKNUM - 1] = 0;
  if(saveFAT()) return FAIL;
  
  return SUCCESS;   // OK
}



//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create( char name[]){  // create new file, make it empty 
  int dirPt = 1;  // dir = 0 reserved for free space
  for (dirPt = 1; dirPt < DIRBLOCKNUM*42; dirPt++)
  {
    if(strncmp(name, directory[dirPt].filename, 8) == 0) return FAIL;  //ditect duplicated name
    if(directory[dirPt].filename[0] == 0) break;  //find the first unused space
  }
  for(int i=0; i<8; i++) directory[dirPt].filename[i] = name[i];
  int freeHead = directory[0].startBlock;
  directory[dirPt].startBlock = freeHead;
  directory[dirPt].usedBytesNum = 0;
  directory[0].startBlock = FAT[freeHead];  //get the next free block in FAT
  FAT[freeHead] = 0;
  if(saveDIR()) return FAIL;
  if(saveFAT()) return FAIL;
  return SUCCESS;
}

//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)

int eFile_WOpen(char name[]){      // open a file for writing 
  if(filePt) return FAIL;
  flagRead = 0;
  for(filePt = 1; filePt < DIRBLOCKNUM*42; filePt++)
  {
    if(strncmp(name, directory[filePt].filename, 8) == 0) break;
    if(filePt == (DIRBLOCKNUM*42 - 1)) return FAIL; //cannot locate the filename
  }
  writePt = directory[filePt].startBlock;
  while(FAT[writePt]) writePt = FAT[writePt]; //the last pt of current file
  if(eDisk_ReadBlock(filebuffer,writePt)) return FAIL;
  return SUCCESS;   
}

//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Write(char data){
  if(!filePt || (filePt && flagRead)) return FAIL;
  if(directory[filePt].usedBytesNum == 512)
  {
    if(eDisk_WriteBlock(filebuffer, writePt)) return FAIL;
    int freeHead = directory[0].startBlock;
    FAT[writePt] = freeHead;
    directory[0].startBlock = FAT[freeHead];  //get the next free block in FAT
    FAT[freeHead] = 0;
    directory[filePt].usedBytesNum = 0;
    writePt = freeHead;
    if(eDisk_ReadBlock(filebuffer,writePt)) return FAIL;
  }
  filebuffer[directory[filePt].usedBytesNum] = data;
  directory[filePt].usedBytesNum ++;
  return SUCCESS;  
}


//---------- eFile_Close-----------------
// Deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently open)
int eFile_Close(void){ 
  if(saveDIR()) return FAIL;
  if(saveFAT()) return FAIL;
  eFile_WClose();
  eFile_RClose();
  return SUCCESS;     
}

//---------- eFile_WClose-----------------
// close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WClose(void){ // close the file for writing
  if(!filePt || (filePt && flagRead)) return FAIL;
  if(eDisk_WriteBlock(filebuffer, writePt)) return FAIL;
  if(saveDIR()) return FAIL;
  if(saveFAT()) return FAIL;
  filePt = 0;
  return SUCCESS;     
}


//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM 
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)

int eFile_ROpen( char name[]){      // open a file for reading 
  if(filePt) return FAIL;
  flagRead = 1;
  for(filePt = 1; filePt < DIRBLOCKNUM*42; filePt++)
  {
    if(strncmp(name, directory[filePt].filename, 8) == 0) break;
    if(filePt == (DIRBLOCKNUM*42 - 1)) return FAIL; //cannot locate the filename
  }
  readPt = directory[filePt].startBlock;
  if(eDisk_ReadBlock(filebuffer,readPt)) return FAIL;
  readIndex = 0;
  return SUCCESS;     
}
 
//---------- eFile_ReadNext-----------------
// retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int eFile_ReadNext( char *pt){       // get next byte 
  if(!filePt || (filePt && !flagRead)) return FAIL;
  if(readIndex >= 512)
  {
    if(FAT[readPt] == 0) return FAIL;
    readPt = FAT[readPt];
    if(eDisk_ReadBlock(filebuffer,readPt)) return FAIL;
    readIndex = 0;
  }
  if(FAT[readPt] == 0 && readIndex >= directory[filePt].usedBytesNum) {
    //readIndex = 0; 
    return 2;
  }
  *pt = filebuffer[readIndex];
  readIndex ++;
  return SUCCESS; 
}

    
//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void){ // close the file for writing
  if(!filePt || (filePt && !flagRead)) return FAIL;
  filePt = 0;
  return SUCCESS;
}




//---------- eFile_Directory-----------------
// Display the directory with filenames and sizes
// Input: pointer to a function that outputs ASCII characters to display
// Output: none
//         0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_Directory(void(*fp)(char)){   
  
  char s[20];
  
  for(int i=1; i<DIRBLOCKNUM*42; i++)
  {
    if(directory[1].filename[0] == 0){
      fp('\n');
      printf("No file in the directory \n");
      break;
    }
    if(directory[i].filename[0])
    {
      for(int j=0; i<8; j++)
      {
        if(!directory[i].filename[j]) break;
        fp(directory[i].filename[j]);
      }
      for(int x=0; x<3; x++) fp('-');
      int size = directory[i].usedBytesNum;
      int fileBlockHead = directory[i].startBlock;
      while(FAT[fileBlockHead]) 
      {
        fileBlockHead = FAT[fileBlockHead];
        size += 512;
      }
      sprintf(s,"size = %d",size);
      for(int i = 0; i < strlen(s); i ++)
      {
        fp(s[i]);
      }
      
      fp('\n');
    }
  }
  return SUCCESS;
}

int eFile_Print(char *string, void(*fp)(char)){   
  if(eFile_ROpen(string)) 
  {
    printf("eFile_ROpen error\n"); 
    return FAIL;
  }
  char data;
  while(!eFile_ReadNext(&data)){
    fp(data);
  }
  //if(eFile_ReadNext(&data))return FAIL;

  eFile_RClose();
  return SUCCESS;
}

//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)


int eFile_Delete( char name[]){  // remove this file 
  for(filePt = 1; filePt < DIRBLOCKNUM*42; filePt++){
    if(strncmp(name, directory[filePt].filename, 8) == 0) break;
    if(filePt == (DIRBLOCKNUM*42 - 1)) return FAIL; //cannot locate the filename
  }
  directory[filePt].filename[0] = 0; //same as format
  int freeHead = directory[0].startBlock;
  directory[0].startBlock = directory[filePt].startBlock;
  int delPt = directory[filePt].startBlock;
  while(FAT[delPt]) delPt = FAT[delPt]; //the last pt of current file
  FAT[delPt] = freeHead;
  if(saveDIR()) return FAIL;
  if(saveFAT()) return FAIL;
  return SUCCESS;    // restore directory back to flash
}

int StreamToFile=0;                // 0=UART, 1=stream to file

int eFile_RedirectToFile(char *name){
  eFile_Create(name);              // ignore error if file already exists
  if(eFile_WOpen(name)) return 1;  // cannot open file
  StreamToFile = 1;
  return 0;
}

int eFile_EndRedirectToFile(void){
  StreamToFile = 0;
  if(eFile_WClose()) return 1;    // cannot close file
  return 0;
}

int fputc (int ch, FILE *f) { 
  if(StreamToFile){
    if(eFile_Write(ch)){          // close file on error
       eFile_EndRedirectToFile(); // cannot write to file
       return 1;                  // failure
    }
    return 0; // success writing
  }

   // regular UART output
  UART_OutChar(ch);
  return 0; 
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);            // echo
  return ch;
}
