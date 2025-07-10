/*
 * filesystem_shell_commands.c
 *
 *  Created on: Jul 10, 2025
 *      Author: gustavo
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "fatfs.h"
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

DIR dir;
FILINFO Finfo;
static BaseType_t prvLSCommand( char *pcWriteBuffer,
                                size_t xWriteBufferLen,
								const char *pcCommandString )
{
    //static BaseType_t state = 0;
	FRESULT f_res;
	uint32_t  p1, s1, s2;
	int len;
	FATFS   *fs;				// Pointer to file system object*/
    char *path = SDPath;

    //if (!state){
        // Abre o diretório
    	p1 = s1 = s2 = 0;
    	f_res = f_opendir(&dir, ".");
        if (f_res == FR_OK) {

            for(;;)
			{
				f_res = f_readdir(&dir, &Finfo);
				if ((f_res != FR_OK) || !Finfo.fname[0]) break;
				if (Finfo.fattrib & AM_DIR)
				{
					s2++;
				} else
				{
					s1++;
					p1 += Finfo.fsize;
				}
				len = sprintf(pcWriteBuffer,"%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s",
						(Finfo.fattrib & AM_DIR) ? 'D' : '-',
						(Finfo.fattrib & AM_RDO) ? 'R' : '-',
						(Finfo.fattrib & AM_HID) ? 'H' : '-',
						(Finfo.fattrib & AM_SYS) ? 'S' : '-',
						(Finfo.fattrib & AM_ARC) ? 'A' : '-',
						(Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
						(Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63,
						Finfo.fsize, &(Finfo.fname[0]));
				pcWriteBuffer += len;

			  Finfo.fname[0] = 0;

		#if _USE_LFN
				len = sprintf(pcWriteBuffer, "  %s\n\r", Finfo.fname);
				pcWriteBuffer += len;
		#else
				printf("\n\r");
		#endif
			}

			len = sprintf(pcWriteBuffer, "%4lu File(s), %lu bytes total \n\r%4lu Dir(s)", s1, p1, s2);
			pcWriteBuffer += len;
			if (f_getfree(path, (DWORD*)&p1, &fs) == FR_OK)
			{
				len = sprintf(pcWriteBuffer, ", %lu bytes free \n\r", p1 * fs->csize * 512);
			}

			f_closedir(&dir);
		} else {
			sprintf(pcWriteBuffer, "Erro ao abrir diretório: %d\n", f_res);
		}

        //return pdTRUE;
        return pdFALSE;
        /*
    }else{
        state = 0;
        strcpy(pcWriteBuffer, "\n\r");
        return pdFALSE;
    }
    */
}

static BaseType_t prvMountCommand( char *pcWriteBuffer,
                                size_t xWriteBufferLen,
								const char *pcCommandString ){
	if (f_mount(&SDFatFS, SDPath, 1) == FR_OK){
        (void)sprintf(pcWriteBuffer, "SD card %s mounted!\r\n", SDPath);
	}else{
		(void)sprintf(pcWriteBuffer, "Failed to mount SD card %s!\r\n", SDPath);
	}
	return pdFALSE;
}

static BaseType_t prvUnmountCommand( char *pcWriteBuffer,
                                size_t xWriteBufferLen,
								const char *pcCommandString ){
	if (f_mount(NULL, SDPath, 1) == FR_OK){
        (void)sprintf(pcWriteBuffer, "SD card %s unmounted!\r\n", SDPath);
	}else{
		(void)sprintf(pcWriteBuffer, "Fail to unmount SD card %s\r\n", SDPath);
	}
	return pdFALSE;
}

static BaseType_t prvReadCommand( char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString ){
	const char *filename = NULL;
	BaseType_t xParameter1StringLength;
	static uint32_t  p1, p2, s2;
	static uint32_t state = 0;
	static uint16_t cnt = 0;
	static TickType_t start_time;
	char buffer[512];

	switch (state) {
		case 0:
			goto open_file;
			break;
		case 1:
			goto read_file;
			break;
	}

	open_file:
		state = 1;
    /* Obtain the name of the source file, and the length of its name, from
       the command string. The name of the source file is the first parameter. */
	filename = FreeRTOS_CLIGetParameter( pcCommandString,
                                         1,
                                         &xParameter1StringLength );
	if (filename != NULL){
		if (f_open(&SDFile, filename, FA_READ) == FR_OK)
		{
			p2 = 0;
			start_time = xTaskGetTickCount();
			p1 = f_size(&SDFile);

			while (p1)
			{
				if (p1 >= sizeof(buffer))
				{
					cnt = sizeof(buffer);
					p1 -= sizeof(buffer);
				}
				else
				{
					cnt = (uint16_t)p1;
					p1 = 0;
				}
				if (f_read(&SDFile, buffer, cnt, (unsigned int *)&s2) != FR_OK)
				{
					break;
				}else
				{
					p2 += s2;
					if (cnt != s2) break;
					(void)memcpy(pcWriteBuffer, buffer, s2);
					pcWriteBuffer += s2;
				}
				return pdTRUE;
				read_file:
			}

			TickType_t stop_time = xTaskGetTickCount();
	        f_close(&SDFile);

	        sprintf(pcWriteBuffer, "\n\r%lu bytes read with %lu bytes/sec.\n\r", p2, (p2 * 1000 / (stop_time - start_time)));
	        state = 0;
	      }else{
	    	  sprintf(pcWriteBuffer, "%s does not exist!\r\n", filename);
	      }
	}else {
		sprintf(pcWriteBuffer, "You need to specify a filename!\r\n");
		state = 0;
	}

    /* There is only a single line of output produced in all cases. pdFALSE is
       returned because there is no more output to be generated. */
    return pdFALSE;
}

static BaseType_t prvDeleteCommand( char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString )
{
	const char *filename;
	BaseType_t xParameter1StringLength;

    /* Obtain the name of the source file, and the length of its name, from
       the command string. The name of the source file is the first parameter. */
	filename = FreeRTOS_CLIGetParameter( pcCommandString,
                                         1,
                                         &xParameter1StringLength );
	if (filename != NULL){
		if (f_unlink(filename) == FR_OK){
			sprintf(pcWriteBuffer, "%s deleted!\r\n", filename);
		}else{
			sprintf(pcWriteBuffer, "%s does not exist!\r\n", filename);
		}
	}else {
		sprintf(pcWriteBuffer, "You need to specify a filename!\r\n");
	}

	return pdFALSE;
}

static BaseType_t prvCreateFileCommand( char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString )
{
	const char *filename;
	BaseType_t xParameter1StringLength;

    /* Obtain the name of the source file, and the length of its name, from
       the command string. The name of the source file is the first parameter. */
	filename = FreeRTOS_CLIGetParameter( pcCommandString,
                                         1,
                                         &xParameter1StringLength );
	if (filename != NULL){
		if (f_open(&SDFile, filename, FA_CREATE_NEW) == FR_OK)
		{
			f_close(&SDFile);
			sprintf(pcWriteBuffer, "%s created!\r\n", filename);
		}else{
			sprintf(pcWriteBuffer, "Unable to create %s!\r\n", filename);
		}
	}else {
		sprintf(pcWriteBuffer, "You need to specify a filename!\r\n");
	}

	return pdFALSE;
}

static BaseType_t prvCreateDirCommand( char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString )
{
	const char *dirname;
	BaseType_t xParameter1StringLength;

    /* Obtain the name of the source file, and the length of its name, from
       the command string. The name of the source file is the first parameter. */
	dirname = FreeRTOS_CLIGetParameter( pcCommandString,
                                         1,
                                         &xParameter1StringLength );
	if (dirname != NULL){
		if (f_mkdir(dirname) == FR_OK)
		{
			sprintf(pcWriteBuffer, "%s created!\r\n", dirname);
		}else{
			sprintf(pcWriteBuffer, "Unable to create %s!\r\n", dirname);
		}
	}else {
		sprintf(pcWriteBuffer, "You need to specify a directory name!\r\n");
	}

	return pdFALSE;
}

static BaseType_t prvChangeDirCommand( char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString )
{
	const char *dirname;
	BaseType_t xParameter1StringLength;

    /* Obtain the name of the source file, and the length of its name, from
       the command string. The name of the source file is the first parameter. */
	dirname = FreeRTOS_CLIGetParameter( pcCommandString,
                                         1,
                                         &xParameter1StringLength );
	if (dirname != NULL){
		if (f_chdir(dirname) != FR_OK)
		{
			sprintf(pcWriteBuffer, "%s does not exist!\r\n", dirname);
		}else
		{
			sprintf(pcWriteBuffer, "\r\n");
		}
	}else {
		sprintf(pcWriteBuffer, "You need to specify a directory name!\r\n");
	}

	return pdFALSE;
}

static BaseType_t prvRenameCommand( char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString )
{
	char filename[128];
	const char *filename_old;
	const char *filename_new;
	BaseType_t xParameterStringLength;

    /* Obtain the name of the source file, and the length of its name, from
       the command string. The name of the source file is the first parameter. */
	filename_old = FreeRTOS_CLIGetParameter( pcCommandString,
                                         1,
                                         &xParameterStringLength );
	memcpy(filename, filename_old, xParameterStringLength);
	filename[xParameterStringLength] = '\0';

	filename_new = FreeRTOS_CLIGetParameter( pcCommandString,
                                         2,
                                         &xParameterStringLength );
	if (filename_old != NULL){
		if (filename_new != NULL){
			if (f_rename(filename, filename_new) == FR_OK){
				sprintf(pcWriteBuffer, "%s moved to %s!\r\n", filename, filename_new);
			}else{
				sprintf(pcWriteBuffer, "%s does not exist!\r\n", filename);
			}
		}else{
			sprintf(pcWriteBuffer, "You need to specify the new filename!\r\n");
		}
	}else {
		sprintf(pcWriteBuffer, "You need to specify the old and new filenames!\r\n");
	}

	return pdFALSE;
}

const CLI_Command_Definition_t xLsCommand =
{
    "ls",
	"ls: List files into the SD card\r\n",
	prvLSCommand,
    0
};

const CLI_Command_Definition_t xMountCommand =
{
    "mount",
	"mount: Mount SD Card\r\n",
	prvMountCommand,
    0
};

const CLI_Command_Definition_t xUnMountCommand =
{
    "unmount",
	"unmount: Unmount SD Card\r\n",
	prvUnmountCommand,
    0
};

const CLI_Command_Definition_t xReadCommand =
{
    "cat",
	"cat: read a file in the shell (parameter: file name)\r\n",
	prvReadCommand,
    1
};

const CLI_Command_Definition_t xDeleteCommand =
{
    "rm",
	"rm: delete a file (parameter: file name)\r\n",
	prvDeleteCommand,
    1
};

const CLI_Command_Definition_t xCreateFileCommand =
{
    "mkfile",
	"mkfile: create an empty file (parameter: file name)\r\n",
	prvCreateFileCommand,
    1
};

const CLI_Command_Definition_t xCreateDirCommand =
{
    "mkdir",
	"mkdir: create a directory (parameter: directory name)\r\n",
	prvCreateDirCommand,
    1
};

const CLI_Command_Definition_t xChangeDirCommand =
{
    "cd",
	"cd: change to a specific directory (parameter: directory name)\r\n",
	prvChangeDirCommand,
    1
};

const CLI_Command_Definition_t xRenameCommand =
{
    "mv",
	"mv: move a file to another name (parameters: old file name and new file name)\r\n",
	prvRenameCommand,
    2
};

void install_fs_shell_commands(void){
	FreeRTOS_CLIRegisterCommand(&xMountCommand);
	FreeRTOS_CLIRegisterCommand(&xUnMountCommand);
	FreeRTOS_CLIRegisterCommand(&xLsCommand);
	FreeRTOS_CLIRegisterCommand(&xReadCommand);
	FreeRTOS_CLIRegisterCommand(&xDeleteCommand);
	FreeRTOS_CLIRegisterCommand(&xCreateFileCommand);
	FreeRTOS_CLIRegisterCommand(&xCreateDirCommand);
	FreeRTOS_CLIRegisterCommand(&xChangeDirCommand);
	FreeRTOS_CLIRegisterCommand(&xRenameCommand);
}

