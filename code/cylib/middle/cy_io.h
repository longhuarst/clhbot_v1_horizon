#ifndef __cy_io_h
#define __cy_io_h












typedef struct {

	int a;
}cy_io_type;









extern cy_io_type cy_io_;











void cy_io_init(void);
void cy_io_polling(void);








#endif //__cy_io_h
