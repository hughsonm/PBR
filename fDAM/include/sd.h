#ifndef SD_H
#define SD_H

int16_t SD_InitHW(void);
int16_t SD_InitCard(void);
int16_t	SD_DiskStatud(void);
int16_t	SD_DiskRead(char * buff, uint16_t sector, uint16_t count);
int16_t SD_DiskWrite(char * buff, uint16_t sector, uint16_t count);

#endif
