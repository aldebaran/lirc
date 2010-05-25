/*      $Id: hw_i2cuser.c,v 5.2 2008/06/30 16:15:04 lirc Exp $      */

/*
 * Remote control driver for I2C-attached devices from userspace
 * Nao's Head IR driver
 *
 * Copyright 2006, 2007 Adam Sampson <ats@offog.org>
 *
 * This does the same job as the lirc_i2c kernel driver and hw_default, but
 * entirely in userspace -- which avoids the need to build the kernel module,
 * and should make this easier to port to other OSs in the future.
 *
 * At the moment, this only supports plain Hauppauge cards, since that's what
 * I've got. To add support for more types of devices, look at lirc_i2c.
 *
 * Based on:
 * Remote control driver for the Creative iNFRA CDrom
 *   by Leonid Froenchenko <lfroen@il.marvell.com>
 * i2c IR lirc plugin for Hauppauge and Pixelview cards
 *   Copyright (c) 2000 Gerd Knorr <kraxel@goldbach.in-berlin.de>
 * Userspace (libusb) driver for ATI/NVidia/X10 RF Remote.
 *   Copyright (C) 2004 Michael Gold <mgold@scs.carleton.ca>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <time.h>
#include <signal.h>

#include <sys/ipc.h>
#include <sys/shm.h>
#define SHMSZ     27

//#include <linux/i2c-dev.h>

#ifndef I2C_SLAVE /* hack */
//#include <linux/i2c.h>
#endif

#include "hardware.h"
#include "ir_remote.h"
#include "lircd.h"
#include "receive.h"
#include "transmit.h"

#include "i2c-dev.h"

/* The number of bits and bytes in a code. */
#define CODE_SIZE_BITS 25
#define CODE_SIZE 4

/* The I2C address of the device to use. */
#define IR_ADDR 8
#define REGISTER 0x44

#define REGISTER_PS_TO_EMIT_COUNT 230
#define REGISTER_PS_TO_EMIT_INFO 231
#define REGISTER_PS_TO_EMIT_0 50

#define REGISTER_PS_PICKED_UP_COUNT 225
#define REGISTER_PS_PICKED_UP_INFO 226
#define REGISTER_PS_PICKED_UP_0 50


static int i2cuser_init(void);
static int i2cuser_deinit(void);
static void i2cuser_read_loop(int fd);
static char *i2cuser_rec(struct ir_remote *remotes);
static lirc_t i2cuser_readdata(lirc_t timeout);
int i2cuser_send(struct ir_remote *remote,struct ir_ncode *code);

struct hardware hw_i2cuser = {
  NULL,                   /* determine device by probing */
  -1,                     /* fd */
  LIRC_CAN_REC_MODE2,  /* features */
  LIRC_CAN_SEND_MODE2, /* send_mode */
  LIRC_MODE_MODE2,     /* rec_mode */
  CODE_SIZE_BITS,         /* code_length */
  i2cuser_init,           /* init_func */
  NULL,                   /* config_func */
  i2cuser_deinit,         /* deinit_func */
  i2cuser_send,                   /* send_func */
  i2cuser_rec,            /* rec_func */
  receive_decode,         /* decode_func */
  NULL,                   /* ioctl_func */
  i2cuser_readdata,
  "i2cuser"
};

/* FD of the i2c device. Since it's not selectable, we give the lircd core a
   pipe and poll it ourself in a separate process. */
static int i2c_fd = -1;
/* The device name. */
char device_name[256];
/* PID of the child process. */
static pid_t child = -1;

/* Hunt for the appropriate i2c device and open it. */
static int open_i2c_device(void) {
  int found=0;
  snprintf(device_name, sizeof device_name, "/dev/i2c-%d", found);
  logprintf(LOG_INFO, "Using i2c device %s", device_name);
  hw.device = device_name;
  return open(device_name, O_RDWR);
}

static int i2cuser_init(void) {
  int pipe_fd[2] = { -1, -1 };

  if (pipe(pipe_fd) != 0) {
    logprintf(LOG_ERR, "Couldn't open pipe: %s", strerror(errno));
    return 0;
  }
  hw.fd = pipe_fd[0];

  i2c_fd = open_i2c_device();
  if (i2c_fd == -1) {
    logprintf(LOG_ERR, "i2c device cannot be opened");
    goto fail;
  }

  if (ioctl(i2c_fd, I2C_SLAVE, IR_ADDR) < 0) {
    logprintf(LOG_ERR, "Cannot set i2c address %02x", IR_ADDR);
    goto fail;
  }

  child = fork();
  if (child == -1) {
    logprintf(LOG_ERR, "Cannot fork child process: %s",
              strerror(errno));
    goto fail;
  } else if (child == 0) {
    close(pipe_fd[0]);
    i2cuser_read_loop(pipe_fd[1]);
  }
  close(pipe_fd[1]);

  logprintf(LOG_INFO, "i2cuser driver: i2c device found and ready to go");
  return 1;

fail:
  if (i2c_fd != -1)
    close(i2c_fd);
  if (pipe_fd[0] != -1)
    close(pipe_fd[0]);
  if (pipe_fd[1] != -1)
    close(pipe_fd[1]);
  return 0;
}

static int i2cuser_deinit(void) {
  if (child != -1) {
    if (kill(child, SIGTERM) == -1)
      return 0;
    if (waitpid(child, NULL, 0) == 0)
      return 0;
  }
  if (i2c_fd != -1)
    close(i2c_fd);
  if (hw.fd != -1)
    close(hw.fd);
  return 1;
}


static void i2cuser_read_loop(int out_fd) {
  unsigned char codeBuf[CODE_SIZE];
  unsigned long int lastTime = 0, timeValue,error=0;
  unsigned int count=0, state=0, info=0, count32=0, d_count=0;
  int i, ret;
  char resetcount[1];
  resetcount[0] = 0;
  unsigned int delay_polling = 50000;
  //FILE *f1;
  //f1=fopen("pulselog","a+");
  //char bufx[128] = "test wouahou !!!\n";

  unsigned char values[32], command = REGISTER;
  alarm(0);
  signal(SIGTERM, SIG_DFL);
  signal(SIGPIPE, SIG_DFL);
  signal(SIGINT, SIG_DFL);
  signal(SIGHUP, SIG_IGN);
  signal(SIGALRM, SIG_IGN);


  char ch_irSide[2] = "8";

   int shmid;
  key_t key;
  char *shm_irSide;

  /*
   * We need to get the segment named
   * "5678", created by the server.
   */
  key = KEY_SHM_IRSIDE;

  /*
   * Locate the segment.
   */
  if ((shmid = shmget(key, SHMSZ, 0666)) < 0) {
      perror("shmget");
      exit(1);
  }

  /*
   * Now we attach the segment to our data space.
   */
  if ((shm_irSide = shmat(shmid, NULL, 0)) == (char *) -1) {
      perror("shmat");
      exit(1);
  }




  //i2c_smbus_write_block_data(i2c_fd, REGISTER_PS_PICKED_UP_COUNT, 1, &(resetcount[0])); //Avoid receiving previous values from buffers

  for (;;) {

    if(count==0) usleep(50000);
    else usleep(10000);

    count = i2c_smbus_read_byte_data(i2c_fd, REGISTER_PS_PICKED_UP_COUNT);
    count32=count;
    d_count=0;

    for (;;) {

      if(count32==0)break;

      info = i2c_smbus_read_byte_data(i2c_fd, REGISTER_PS_PICKED_UP_INFO);

      //first we have to read the informational byt
      //if(count32 == count)
      //{
        if(info&0x10) state=1;
        else state=0;
      //}

      

      if(count32<=32)
      {
        i2c_smbus_read_i2c_block_data(i2c_fd, REGISTER_PS_PICKED_UP_0 + d_count, count32, &(values[d_count]));
        
        if((info!=255) && (values[0]!=0)){  //
          *shm_irSide = info>>5;
          info=255;        
        } 
      }
      else
      {
        i2c_smbus_read_i2c_block_data(i2c_fd, REGISTER_PS_PICKED_UP_0 + d_count, 32, &(values[d_count]));
      }
      
      //logprintf(LOG_INFO, "DCOUNT: %ud / COUNT32: %ud / COUNT: %ud",d_count,count32, count);
      for(i=0;i<count32;i++){
        //logprintf(LOG_INFO, "INDICE1: %d; VAL: %d",i,values[i]);
        if(values[i]==0){
          lastTime=lastTime+8160;
        }else{
          //logprintf(LOG_INFO, "INDICE2: %d",i);
          //if((i==0) && (d_count==0)) timeValue=200000;
          //else 
          timeValue=lastTime+(unsigned long int)values[i]*32;
          codeBuf[0]= timeValue & 0xFF;
          codeBuf[1]= (timeValue & 0xFF00)>>8;
          codeBuf[2]= (timeValue & 0xFF0000)>>16;
          codeBuf[3]= state;//to save if it is a pulse or a duration
          if (write(out_fd, codeBuf, CODE_SIZE) != CODE_SIZE) {//write on pipe
            logprintf(LOG_ERR, "Write to i2cuser pipe failed: %s",
                      strerror(errno));
            goto fail;
          }

          error++;
          lastTime=0;
          if(state==1)state=0;
          else state=1;
        }
        if(error>128)break;
      }

      if(error>128)break;
      if(count32<=32)
      {
        count32=0;
        break;
      }
      else count32 -=32;
      d_count = count-count32;
      
    }
    error=0;
    if(count!=0) i2c_smbus_write_block_data(i2c_fd, REGISTER_PS_PICKED_UP_COUNT, 1, &(resetcount[0]));
  }
  //fclose(f1);
  fail:
  _exit(1);
}


static char *i2cuser_rec(struct ir_remote *remotes) {
  logprintf(LOG_INFO, "i2cuser_rec");
  if (!clear_rec_buffer()) return NULL;
  return decode_all(remotes);
}

static lirc_t i2cuser_readdata(lirc_t timeout)
{
  int n;
  lirc_t res = 0;

  while(res==0){
    if (!waitfordata(timeout))
    {
      res= 0;
    }
    n = read(hw.fd, &res, CODE_SIZE);
    if (n != CODE_SIZE)
    {
      res = 0;
    }
  }
  return(res);
}

int i2cuser_send(struct ir_remote *remote,struct ir_ncode *code){
  int length, x=0,i, ret, sent=0, sent_confirmed=0;// lengthI2C=0;
  lirc_t *signals,val=0,pulseState=0;

  unsigned char values[34], command = REGISTER;
  unsigned char values_init[5];

  if(!init_send(remote,code)) {
    return 0;
  }

  length = send_buffer.wptr;//longueur totale
  signals = send_buffer.data;


  values_init[0] = REGISTER_PS_TO_EMIT_COUNT;
  values_init[1] = 2;
  values_init[2] = length;


  for(;;){

    sent_confirmed = sent;

    //fill array to send
    for(i=0;(i<32)&&(x<(sent+length));i++){//parcourir tableau à envoyer
      if(val==0){
        val = (signals[x]&PULSE_MASK);
        if(pulseState==0) pulseState=1;
        else pulseState=0;
        //logprintf(LOG_INFO, "pulse to send=  %d", val);
      }

      if(i==0){
            if(pulseState) values_init[3]=0x10;//information byte
            else values_init[3]=0x00;
      }


      if(val>8160){//time out à envoyer
        values[2+i]=0;
        val = val-8160;
        values_init[2]++;
      }else{
        values[2+i]=val/32;//valeur inférieure a 8160 ms
        if((val%32)>16) values[2+i] += 1;
        val=0;
        x++;
        length--;
        sent++;
      }
      logprintf(LOG_INFO, "pulse to send=  %d", values[2+i]);
    }

    values[1]=i;

    logprintf(LOG_INFO, "x=  %d, sent+length= %d", x,sent+length);
    if(x<(sent+length))
      values_init[3]=values_init[3] | 0x1;



    //logprintf(LOG_INFO, "Send Count + Info bytes");
    //ret=write(i2c_fd,values_init,values_init[1]+2);
    //if(ret== -1) return 0;

    i2c_smbus_write_block_data(i2c_fd, values_init[0], values_init[1], &(values_init[2]));

    //logprintf(LOG_INFO, "Send Data");
    values[0] = REGISTER_PS_TO_EMIT_0 + sent_confirmed;
    i2c_smbus_write_block_data(i2c_fd, values[0], values[1], &(values[2]));

    //for(i=0;i<values[1]+2;i++){
      //logprintf(LOG_INFO, "val[%d]=%d",i,values[i]);
    //}


    if(x>=(sent+length)) break;
  }
  return 1;
}
