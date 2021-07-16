#ifndef __AICE_JDP_H__
#define __AICE_JDP_H__

/* JDP Instructions */
#define JDP_READ	  0x00
#define JDP_WRITE	  0x80

#define ACCESS_DIM          0x01        //4'b0001
#define ACCESS_DBG_SR       0x02        //4'b0010
#define ACCESS_DTR          0x03        //4'b0011
#define ACCESS_MEM_W        0x04        //4'b0100
#define ACCESS_MISC_REG     0x05        //4'b0101
#define FAST_ACCESS_MEM     0x06        //4'b0110
#define GET_DBG_EVENT       0x07        //4'b0111
#define EXECUTE             0x08        //4'b1000
#define IDCODE              0x09        //4'b1001
#define ACCESS_MEM_H        0x0A        //4'b1010
#define ACCESS_MEM_B        0x0B        //4'b1011
#define BYPASS              0x0F        //4'b1111


/* JDP Read Instructions */
#define JDP_R_DIM       (JDP_READ | ACCESS_DIM)
#define JDP_R_DBG_SR    (JDP_READ | ACCESS_DBG_SR)
#define JDP_R_DTR       (JDP_READ | ACCESS_DTR)
#define JDP_R_MEM_W     (JDP_READ | ACCESS_MEM_W)
#define JDP_R_MISC_REG  (JDP_READ | ACCESS_MISC_REG)
#define JDP_R_FAST_MEM  (JDP_READ | FAST_ACCESS_MEM)
#define JDP_R_DBG_EVENT (JDP_READ | GET_DBG_EVENT)
#define JDP_R_IDCODE    (JDP_READ | IDCODE)
#define JDP_R_MEM_H     (JDP_READ | ACCESS_MEM_H)
#define JDP_R_MEM_B     (JDP_READ | ACCESS_MEM_B)


/* JDP Write Instructions */
#define JDP_W_DIM       (JDP_WRITE | ACCESS_DIM)
#define JDP_W_DBG_SR	(JDP_WRITE | ACCESS_DBG_SR)
#define JDP_W_DTR		(JDP_WRITE | ACCESS_DTR)
#define JDP_W_MEM_W		(JDP_WRITE | ACCESS_MEM_W)
#define JDP_W_MISC_REG  (JDP_WRITE | ACCESS_MISC_REG)
#define JDP_W_FAST_MEM	(JDP_WRITE | FAST_ACCESS_MEM)
#define JDP_W_EXECUTE	(JDP_WRITE | EXECUTE)
#define JDP_W_MEM_H		(JDP_WRITE | ACCESS_MEM_H)
#define JDP_W_MEM_B		(JDP_WRITE | ACCESS_MEM_B)


#endif // __AICE_JDP_H__
