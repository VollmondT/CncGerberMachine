#ifndef _GERBER_H
#define _GERBER_H

#include <stdint.h>

#define TOOL_W 20

extern int CUR_X;
extern int CUR_Y;

typedef void (*ApertureFlash)(void *, void *, int, int);
typedef void (*ApertureLineTo)(void *, void *, int, int);
typedef void(*ApertureDtor)(void *);

typedef struct Aperture {
	unsigned code;
	ApertureDtor dtor;
	ApertureFlash flash;
	ApertureLineTo line;
	void* next;
	const char* name;
} Aperture;

typedef struct GerberContext {
	Aperture* apertures;
	Aperture* current_aperture;
	unsigned is_mm;
	unsigned is_absolute_coords;
	int x;
	int y;
	unsigned coords_x_fraq;
	unsigned coords_y_fraq;
	unsigned step_accuracy; // 0.04 mm = 4
	unsigned is_linear_interpolation;
	unsigned is_single_quadrant;
	unsigned is_clockwise;
	unsigned tool_width; // 0.04mm = 4
	unsigned line_counter;
} GerberContext;



GerberContext* GerberContextNew(void);
void GerberContextFree(GerberContext* ctx);

void GerberAcceptCommand(GerberContext* ctx, int argc, char* argv[]);
void MoveTo(const int x, const int y, int silent);
void MoveToRelative(const int x, const int y, int silent);

#endif // _GERBER_H

