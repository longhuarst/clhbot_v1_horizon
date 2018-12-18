#ifndef __cy_bsp_h
#define __cy_bsp_h












typedef struct {

		int a;
}cy_bsp_type;









extern cy_bsp_type cy_bsp_;











void cy_bsp_init(void);
void cy_bsp_polling(void);








#endif //__cy_bsp_h
