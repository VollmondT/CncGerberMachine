#include <stdlib.h>
#include <math.h>
#include "motor.h"
#include "gerber.h"
#include "laser.h"

static BaseSequentialStream *chp = (BaseSequentialStream*)&SD3;

int CUR_X = 0;
int CUR_Y = 0;

typedef struct ApertureC {
	Aperture a;
	unsigned radix;
} ApertureC;

typedef struct ApertureR {
	Aperture a;
	unsigned w,h;
} ApertureR;

typedef struct ApertureO {
	Aperture a;
	unsigned w,h;
} ApertureO;

static int GerberInterpretCoords(GerberContext *ctx, long long in, long long is_x) {
	// TODO use ctx->coords_x_fraq and ctx->coords_y_fraq
	(void)is_x;
	return in / (1000* ctx->step_accuracy);
}

static void ApertureCFlash(GerberContext *ctx, ApertureC *a, int xpos, int ypos) {

	unsigned half_accuracy = 2 * ctx->step_accuracy;
	unsigned tool = ctx->tool_width / half_accuracy;
	if( ctx->tool_width % half_accuracy ) {
		++tool;
	}
	int radix_in_steps = a->radix/ctx->step_accuracy - tool;
	unsigned dir = 0;
	
	for(int y = -radix_in_steps; y <= radix_in_steps; ++y ) {
		for( int x = -radix_in_steps; x <= 0; ++x) {
			if( sqrtf(x*x + y*y) <= radix_in_steps ) {
				if( dir ) {
					MoveTo(xpos - x, ypos + y);
					LaserEnable();
					MoveTo(xpos + x, ypos + y);
					LaserDisable();
					dir++;
				} else {
					MoveTo(xpos + x, ypos + y);
					LaserEnable();
					MoveTo(xpos - x, ypos + y);
					LaserDisable();
					dir--;
				}
				break;
			}
		}
	}
}

static void FillRectangle(int x1, int y1, int x2, int y2, int xlen, int ylen) {
	const int deltaX = abs(x2 - x1);
	const int deltaY = abs(y2 - y1);
	const int signX = x1 < x2 ? 1 : -1;
	const int signY = y1 < y2 ? 1 : -1;
	
	int error = deltaX - deltaY;
	
	//chprintf(chp, "(%d,%d) to (%d,%d) %d %d\r\n", x1, y1, x2, y2, xlen, ylen);
	
	while(x1 != x2 || y1 != y2) {
		MoveTo(x1, y1);
		LaserEnable();
		MoveTo(x1 + xlen, y1 + ylen);
		LaserDisable();

		const int error2 = error * 2;

		if(error2 > -deltaY) {
			error -= deltaY;
			x1 += signX;
		}
		if(error2 < deltaX) {
			error += deltaX;
			y1 += signY;
		}
	}

	MoveTo(x2, y2);
	LaserEnable();
	MoveTo(x2 + xlen, y2 + ylen);
	LaserDisable();
}

static void ApertureCLine(GerberContext *ctx, ApertureC *a, int x, int y) {
	
	unsigned half_accuracy = 2 * ctx->step_accuracy;
	unsigned tool = ctx->tool_width / half_accuracy;
	if( ctx->tool_width % half_accuracy ) {
		++tool;
	}
	int radix_in_steps = a->radix/ctx->step_accuracy;
	if( radix_in_steps > 10 ) {
		ApertureCFlash(ctx, a, ctx->x, ctx->y);
	}
	
	int deltax = (x - ctx->x);
	int deltay = (y - ctx->y);
	
	int A_x_pos, A_y_pos;
	int B_x_pos, B_y_pos;
	
	if( deltax == 0 ) {
		A_x_pos = radix_in_steps - tool;
		A_y_pos = 0;
		B_x_pos = -radix_in_steps + tool;
		B_y_pos = 0;
	} else if (deltay == 0 ) {
		A_x_pos = 0;
		A_y_pos = radix_in_steps - tool;
		B_x_pos = 0;
		B_y_pos = -radix_in_steps + tool;
	} else {
		float tgx_fi_quad = (float)deltay/(float)deltax;
		tgx_fi_quad *= tgx_fi_quad;
		float qy = sqrtf( radix_in_steps * radix_in_steps * tgx_fi_quad / (tgx_fi_quad + 1) );
		float qx = qy / tgx_fi_quad;

		// quadrants
		// | Q4 | Q1 |
		// | Q3 | Q2 |
		if( (deltax > 0 && deltay > 0) || (deltax < 0 && deltay < 0)  ) {
			// Q1 and Q3
			A_x_pos = -qy;
			A_y_pos = qx;
			B_x_pos = qy;
			B_y_pos = -qx;
		} else {
			// Q2 and Q4
			A_x_pos = qy;
			A_y_pos = qx;
			B_x_pos = -qx;
			B_y_pos = -qy;
		}
	}
	
	FillRectangle(ctx->x + A_x_pos, ctx->y + A_y_pos, ctx->x + B_x_pos, ctx->y + B_y_pos, deltax, deltay);
	
	if( radix_in_steps > 10 ) {
		ApertureCFlash(ctx, a, x, y);
	}
}

static Aperture* ApertureCNew(unsigned code, const char* data) {
	ApertureC* a = (ApertureC*)malloc(sizeof(ApertureC));
	a->a.code = code;
	a->a.flash = (ApertureFlash)ApertureCFlash;
	a->a.line = (ApertureLineTo)ApertureCLine;
	a->a.dtor = free;
	a->a.next = NULL;
	float radix;
	if( sscanf(data, "%f", &radix) ) {
		a->radix = radix * 50; // (100/2)
	} else {
		a->radix = 0;
	}
	return (Aperture*)a;
}

static void ApertureRFlash(GerberContext *ctx, ApertureR *a, int xpos, int ypos) {
	// calculate step with tool accuracy
	unsigned half_accuracy = 2*ctx->step_accuracy;
	unsigned tool = ctx->tool_width/half_accuracy;
	if( ctx->tool_width % half_accuracy ) {
		++tool;
	}
	
	int x0 = xpos + tool - a->w/half_accuracy;
	int x1 = xpos - tool + a->w/half_accuracy;
	int y0 = ypos + tool - a->h/half_accuracy;
	int y1 = ypos - tool + a->h/half_accuracy;
	unsigned dir = 0;
	
	MoveTo(x0,y0);
	
	LaserEnable();
	
	if( a->w > a->h ) {
		for( int y = y0; y <=y1; ++y) {
			MoveTo(CUR_X, y);
			if( dir == 0) {
				++dir;
				MoveTo(x1, y);
			} else {
				--dir;
				MoveTo(x0, y);
			}
		}
	} else {
		for( int x = x0; x <=x1; ++x) {
			MoveTo(x, CUR_Y);
			if( dir == 0) {
				++dir;
				MoveTo(x, y1);
			} else {
				--dir;
				MoveTo(x, y0);
			}
		}
	}
	
	LaserDisable();
}

static Aperture* ApertureRNew(unsigned code, const char* data) {
	ApertureR* a = (ApertureR*)malloc(sizeof(ApertureR));
	a->a.code = code;
	a->a.flash = (ApertureFlash)ApertureRFlash;
	a->a.line = NULL;
	a->a.dtor = free;
	a->a.next = NULL;
	float w,h;
	if( sscanf(data, "%fX%f", &w, &h) == 2 ) {
		a->w = w *100;
		a->h = h *100;
	} else {
		a->w = 0;
		a->h = 0;
	}
	return (Aperture*)a;
}

static void ApertureOFlash(GerberContext *ctx, ApertureC *a, int xpos, int ypos) {
	(void)ctx;
	(void)a;
	(void)xpos;
	(void)ypos;
}

static Aperture* ApertureONew(unsigned code, const char* data) {
	ApertureO* a = (ApertureO*)malloc(sizeof(ApertureO));
	a->a.code = code;
	a->a.flash = (ApertureFlash)ApertureOFlash;
	a->a.line = NULL;
	a->a.dtor = free;
	a->a.next = NULL;
	float w,h;
	if( sscanf(data, "%fX%f", &w, &h) == 2 ) {
		a->w = w *100;
		a->h = h *100;
	} else {
		a->w = 0;
		a->h = 0;
	}
	return (Aperture*)a;
}

GerberContext* GerberContextNew(void) {
	GerberContext *ctx = (GerberContext*)malloc(sizeof(GerberContext));
	ctx->apertures = NULL;
	ctx->current_aperture = NULL;
	ctx->is_absolute_coords = 1;
	ctx->is_mm = 1;
	ctx->x = 0;
	ctx->y = 0;
	ctx->coords_x_fraq = 5;
	ctx->coords_y_fraq = 5;
	ctx->step_accuracy = 4;
	
	ctx->is_linear_interpolation = 1;
	ctx->is_single_quadrant = 1;
	ctx->is_clockwise = 1;
	ctx->tool_width = 10;
	return ctx;
}

void GerberContextFree(GerberContext* ctx) {
	while( ctx->apertures ) {
		Aperture *next = (Aperture *)ctx->apertures->next;
		ctx->apertures->dtor(ctx->apertures);
		ctx->apertures = next;
	}
	free(ctx);
}

static void GerberContextAddAperture(GerberContext* ctx, char *data) {
	unsigned code;
	char buf[20];
	char type;

	int ret = sscanf(data, "%d%c,%s", &code, &type, buf);
	if( ret != 3 ) {
		chprintf(chp, "Failed to parse aperture: %s", data);
		return;
	}

	Aperture* a = NULL;
	switch( type ) {
	case 'C':
		a = ApertureCNew(code, buf);
		break;
		
	case 'R':
		a = ApertureRNew(code, buf);
		break;

	case 'O':
		a = ApertureONew(code, buf);
		break;

	default:
		chprintf(chp, "Failed to add unknown aperture: %c", type);
		return;
	}
	
	Aperture *prev = NULL;
	Aperture *current = ctx->apertures;
	while( current != NULL ) {
		prev = current;
		current = (Aperture*)current->next;
	}
	if( prev == NULL ) {
		ctx->apertures = a;
	} else {
		prev->next = a;
	}
}

static void GerberExecuteD(GerberContext* ctx, unsigned code, int x, int y) {
	switch(code) {
	case 1: //D1
		if( ctx->current_aperture ) {
			if( ctx->current_aperture->line ) {
				ctx->current_aperture->line(ctx, ctx->current_aperture, x, y);
			}
		}
		break;
	
	case 2: //D2
		MoveTo(x, y);
		break;

	case 3: //D3
		if( ctx->current_aperture ) {
			if( ctx->current_aperture->flash ) {
				ctx->current_aperture->flash(ctx, ctx->current_aperture, x, y);
			}
		}
		break;

	default:
		chprintf(chp, "unknown D-code: %u\r\n", code);
	}
	// cache it for future use (anyway)
	ctx->x = x;
	ctx->y = y;
}

static void GerberLoadAperture(GerberContext* ctx, unsigned code) {
	Aperture* current = ctx->apertures;
	ctx->current_aperture = NULL;
	while( current != NULL ) {
		if( current->code == code ) {
			ctx->current_aperture = current;
			break;
		}
		current = current->next;
	}
	
	if( ctx->current_aperture == NULL ) {
		chprintf(chp, "failed to set aperture: %u\r\n", code);
	}
}

void GerberAcceptCommand(GerberContext* ctx, int argc, char* argv[]) {
	for( int i = 0; argv[0][i] != '\0'; ++i ) {
		if( argv[0][i] == '*' ) {
			// set command end
			argv[0][i] = '\0'; 
		}
	}
	
	if( strncmp(argv[0], "G04", 3) == 0 ) {
		// ignore comments
	} else if ( strncmp(argv[0], "%ADD", 4) == 0 ) {
		GerberContextAddAperture(ctx, argv[0] + 4);
	} else if ( strncmp(argv[0], "%LPD", 4) == 0 ) {
		// dark polarity, never mind
	} else if ( strncmp(argv[0], "%MOMM", 5) == 0 ) {
		ctx->is_mm = 1;
	} else if ( strncmp(argv[0], "%FSLA", 5) == 0 ) {
		unsigned x, y;
		if( sscanf(argv[0] + 5, "X%uY%u", &x, &y) == 2 ) {
			ctx->coords_x_fraq = x % 10;
			ctx->coords_y_fraq = y % 10;
		} else {
			chprintf(chp, "FSLA failed: %s\r\n", argv[0] + 5);
		}
	} else if ( strncmp(argv[0], "G70", 3) == 0 ) {
		ctx->is_mm = 0;
	} else if ( strncmp(argv[0], "G71", 3) == 0 ) {
		ctx->is_mm = 1;
	} else if ( strncmp(argv[0], "G74", 3) == 0 ) {
		ctx->is_single_quadrant = 1;
	} else if ( strncmp(argv[0], "G75", 3) == 0 ) {
		ctx->is_single_quadrant = 2;
	} else if ( strncmp(argv[0], "G90", 3) == 0 ) {
		ctx->is_absolute_coords = 1;
	} else if ( strncmp(argv[0], "G91", 3) == 0 ) {
		ctx->is_absolute_coords = 0;
	} else if ( strncmp(argv[0], "G01", 3) == 0 ) {
		ctx->is_linear_interpolation = 1;
	} else if ( strncmp(argv[0], "G02", 3) == 0 ) {
		ctx->is_linear_interpolation = 0;
		ctx->is_clockwise = 1;
	} else if ( strncmp(argv[0], "G03", 3) == 0 ) {
		ctx->is_clockwise = 0;
	} else if ( strncmp(argv[0], "M02", 3) == 0 ) {
		MoveTo(0,0); //return to the origin
	} else {
		long long x = 0, y = 0;
		unsigned code;
		if( sscanf(argv[0], "X%lldY%lldD%u", &x, &y, &code) == 3 ) {
			GerberExecuteD(ctx, code, GerberInterpretCoords(ctx, x, 1), GerberInterpretCoords(ctx, y, 0));
			return;
		} else if( sscanf(argv[0], "X%lldD%u", &x, &code) == 2 ) {
			GerberExecuteD(ctx, code, GerberInterpretCoords(ctx, x, 1), ctx->y);
			return;
		} else if( sscanf(argv[0], "Y%lldD%u", &y, &code) == 2 ) {
			GerberExecuteD(ctx, code, ctx->x, GerberInterpretCoords(ctx, y, 0));
			return;
		} else if( sscanf(argv[0], "D%u", &code) == 1 ) {
			GerberLoadAperture(ctx, code);
			return;
		}
		// default (unknown)
		chprintf(chp, "%d %s\r\n", argc, argv[0]);
	}
}

void MoveTo(const int xpos, const int ypos) {
	int x_delta = xpos - CUR_X;
	int y_delta = ypos - CUR_Y;
	//chprintf(chp, "CUR_X=%d CUR_Y=%d deltax=%d deltay=%d\r\n", CUR_X, CUR_Y, x_delta, y_delta);
	
	if( x_delta >= 0 ) {
		MotorDriverSetDirection(MOTOR_X, MOTOR_X_DIRECTION_PLUS);
	} else {
		MotorDriverSetDirection(MOTOR_X, MOTOR_X_DIRECTION_MINUS);
		x_delta = - x_delta;
	}
	if( y_delta >= 0 ) {
		MotorDriverSetDirection(MOTOR_Y, MOTOR_Y_DIRECTION_PLUS);
	} else {
		MotorDriverSetDirection(MOTOR_Y, MOTOR_Y_DIRECTION_MINUS);
		y_delta = - y_delta;
	}
	
	if( x_delta || y_delta ) {
		MotorGroupMakeSteps(x_delta, y_delta);
		CUR_X = xpos;
		CUR_Y = ypos;
	}
}
