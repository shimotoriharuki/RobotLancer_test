/*
 * HAL_SDcard_lib.h
 *
 *  Created on: 2019/10/25
 *      Author: UnderBird
 *
 *  Sample_code
 *
 */


#ifndef HAL_SDCARD_LIB_H
#define HAL_SDCARD_LIB_H

void fopen_folder_and_file(void);
void create_path(char *, char *);

FRESULT sd_mount(void);   					//SDをマウント
FRESULT sd_unmount(void);					  //SDをアンマウント
FRESULT user_fopen(char *, char *);
FRESULT user_fclose(void);
FRESULT sd_write(short, float *, char);
FRESULT sd_read(short, float *);

FRESULT sd_write_array(char *, char * , short, float *, char); 	 //SDに書き込み
FRESULT sd_read_array(char *, char * , short, float *);		//SDから読み込み

FRESULT sd_write_array_int(char *, char * , short, int *, char); 	 //SDに書き込み
FRESULT sd_read_array_int(char *, char * , short, int *);		//SDから読み込み

void bufclear(void);


#endif
