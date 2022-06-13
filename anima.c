
#define DECLARATION

#pragma warning(disable : 4244)     // MIPS
#pragma warning(disable : 4136)     // X86
#pragma warning(disable : 4051)     // ALPHA


#define	WIDTH	1000
#define	HEIGHT	800
//使っていない？

#ifdef LSIZE
#define	WIDTH2		781
#define	HEIGHT2		581

#elif MSIZE
#define	WIDTH2		600
#define	HEIGHT2		450

#elif SSIZE
#define	WIDTH2		450
#define	HEIGHT2		338

#else 
#define	WIDTH2		781
#define	HEIGHT2		581
#endif
//#define	WIDTH2		400
//#define	HEIGHT2		300

//#define	WIDTH2		600
//#define	HEIGHT2		450

//#define	WIDTH2		500
//#define	HEIGHT2		375

//#define	WIDTH2		450
//#define	HEIGHT2		338


//ウインドウの位置
#ifdef LA
#define	WX		1
#define	WY		1

#elif LB
#define	WX		1
#define	WY		390

#elif RA
#define	WX		465
#define	WY		1

#elif RB
#define	WX		465
#define	WY		390

#else
#define	WX		1
#define	WY		1
#endif

//#define  SLOPE_ANGLE	10.0

//#define	WX		580
//#define	WY		450


#define	MAX_FORCE	300.
#define		DIM			3		/* The number of dimension */
#define XX 	0
#define YY 	1
#define ZZ 	2

#define 	PI			3.1415926535
#define		GA			9.8

#define		ON			1
#define		OFF			0
#define		NUM_LINK	50		/* The number of link */
#define		NUM_POINT		5
#define		MAX_LINE	1000
#define		NUM_DIF			160

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <math.h>
#include <stdio.h>
//#include "robotModel.h"
#include "color.h"
#include "uglyfont.h"

long  tm, tmbef=0, tmnow, tmhz=0, itm=0, tmtbl[100] ;

static int cnt,timer;
//static GLdouble eye[3]={2.0,-13.0,0.8};
static GLdouble eye[3]={2.0,-45.0,0.8};
static GLdouble center[3]={2.0,0.0,0.8};

int generateEPS(char *filename, int inColor, unsigned int width,
	unsigned int height);
int	get_model_from_file(char *filename);
void	Eulerian_ang() ;
void	calc_Rot() ;
void	calc_abs_co() ;
double	*mmul31() ;

float	BodyTranslate[]={ 2.0, -6.0, -0.1 };
//int		ALLROTATE[]={ -50, 10 };
int		ALLROTATE[]={ -0, 0 };
int		eps_flg = 0, trans_flg = ON ;
int		data_gc_flg = OFF ;	// 2005.10.06
int		ASC_flg = OFF, POS_flg = OFF ;

int		MARKER_flg = ON, MARKER_LINE_flg = ON ; // 2007.02.08

int		INV_FRONT_flg = OFF ;

int	 	ASC_read_flg = OFF ;
int		stick_flg = OFF ;
int		stick_read_flg = OFF ;

static int Stop=0;
int		loop_flg = OFF ;

static double rt[MAX_LINE][NUM_LINK][DIM][DIM],tr[MAX_LINE][NUM_LINK][DIM];// rot,ori
static double p_mkr[MAX_LINE][NUM_LINK][DIM] ; // position of marker
static double real_time[MAX_LINE] ; // counter time 
static double	g_exf[MAX_LINE][2][DIM], cop[MAX_LINE][2][DIM], COG[MAX_LINE][DIM] ;
static int		num_marker_line, marker_line_s[NUM_LINK], marker_line_f[NUM_LINK] ; // line bet. mkr and mkr
static int num_marker_line_white, num_marker_line_green, num_marker_line_blue ; // line color
static int instantial_axis, grf_marker_line_s[2], grf_marker_line_f[2] ; // rotational axis, grf
//static int grf_marker_line_s[2], grf_marker_line_f[2] ; // rotational axis, grf

static int num_characteristic_marker, char_marker[50] ;



double qXX[MAX_LINE] ;

char	model_name[100]  ;  
double	*add3( double *a, double *b, double *c ) ;


void axis (float d, float s) ;
void subaxis(float d, float s) ;
void myGround(double height) ;

void ReadSkeleton() ;
void ReadAngle() ;

// using examples from http://atlas.riken.jp/~koishi/claret.html
// mouse()
// motion()
int mouse_l = 0;
int mouse_m = 0;
int mouse_r = 0;
//int mpos[2] ;
//int trans[3], angle[3], eye_len ;
int trans[3], eye_len ;
double angle[3], mpos[2] ;

int grf_flg = 0, cog_flg = 0, mscl_flg = 1 ; // flags of Groud Reaction Force, Center of Gravity and Muscle
int flr_solid = 1 ; // flag of floor(Solid/WireFlame)
int jo_ball_flg = 1 ; // joint ball flag
int num_mkr, num_ctr ; // number of marker, number of center
int ctr[20] ; // center marker column order Number
//int num_mkr = 41 ;

double SLOPE_ANGLE = 0.0 ;

int skeletonNo[8] ;
char skeletonName[8][10] ;
double skeletonWeight[8], skeletonLength[8] ;
double skeletonCOGr[8], skeletonMomInertia[8] ;
double body_weight = 0.0 ;
double body_cog[MAX_LINE][2] ;



/*---------------------------*/


/* skeletonデータを読み込む */
void ReadSkeleton(void)
{
	FILE *fp;
	char f_name[64],buffer[20000],*fgets();	
	int i ;	

	cnt = 0 ;


	  sprintf(f_name,"skeleton.csv" );
	  if ((fp=fopen(f_name,"rt"))==NULL) {
		printf("Cannnot open %s\n",f_name);
		exit(0);
	  }

//		printf("sprintf(f_name,skeleton.csv );は終わったよmain.c\n");	//debug用

		cnt = 0 ;
	  for( i = 0 ; i < 10 ; i++ ) {
	   if ( fgets( buffer, 10000, fp ) == NULL ) {
		break ;
	   }
	    if ( buffer[0] != '#' ) {
	     skeletonNo[cnt] = atoi( strtok( buffer, "," ) ) ;
//		printf("No=%d  ", skeletonNo[cnt] );	//debug用
	     sscanf( strtok( NULL, "," ), " %s ", skeletonName[cnt] ) ;
//		printf("Name=%s  ", skeletonName[cnt] );	//debug用
	     skeletonWeight[cnt] = atof( strtok( NULL, "," ) ) ;
	     body_weight += skeletonWeight[cnt] ;
	     skeletonLength[cnt] = atof( strtok( NULL, "," ) ) ;
//		printf("Length=%f\n", skeletonLength[cnt] );	//debug用
	     skeletonCOGr[cnt] = atof( strtok( NULL, "," ) ) ;
	     skeletonMomInertia[cnt] = atof( strtok( NULL, "," ) ) ;
	     cnt++ ;
	    }
	  }

	fclose(fp); 

//	  for( i = 0 ; i < 8 ; i++ ) {
//		printf("No=%d  ", skeletonNo[i] );	//debug用
//		printf("Name=%s  ", skeletonName[i] );	//debug用
//		printf("Length=%f\n", skeletonLength[i] );	//debug用
//	  }

}

/* motionデータを読み込む */
void ReadAngle(void)
{
	FILE *fp;
	char f_name[64],buffer[20000],*fgets();	
	int i,j,k,t,n;	
	double	seg_angle[MAX_LINE][8], body_originx[MAX_LINE], body_originz[MAX_LINE] ;
	double	seg_cog[MAX_LINE][8][2] ;
	int	d ;
	double trx, temp, tempx, tempz ;
	int pelvis_num ;

	cnt = 0 ;


	  sprintf(f_name,"%s.csv", model_name );
	  if ((fp=fopen(f_name,"rt"))==NULL) {
		printf("Cannnot open %s\n",f_name);
		exit(0);
	  }

	//	printf("sprintf(f_name,.motgc, model_name );は終わったよmain.c\n");	//debug用



		num_mkr = 10 ;
		num_ctr = 10 ;

		for ( i = 0 ; i < num_ctr ; i ++ ) {
			ctr[i] = i ;
		}

		num_marker_line = 9 ;

		marker_line_s[0] = 0 ; // head
		marker_line_f[0] = 1 ; 
		marker_line_s[1] = 1 ; // lumber
		marker_line_f[1] = 2 ;  // rhip

		marker_line_s[2] = 2 ;  
		marker_line_f[2] = 3 ;  // rknee 
		marker_line_s[3] = 3 ;
		marker_line_f[3] = 4 ;  // rankle
		marker_line_s[4] = 4 ;
		marker_line_f[4] = 5 ;  // rtoe

		marker_line_s[5] = 6 ;  // lhip
		marker_line_f[5] = 7 ;  // lknee
		marker_line_s[6] = 7 ;  
		marker_line_f[6] = 8 ;  // lankle
		marker_line_s[7] = 8 ;
		marker_line_f[7] = 9 ;  // ltoe

		marker_line_s[8] = 1 ; // lumber
		marker_line_f[8] = 6 ;  // lhip

		grf_marker_line_s[0] = 10 ;
		grf_marker_line_f[0] = 11 ;  // r grf

		grf_marker_line_s[1] = 12 ;
		grf_marker_line_f[1] = 13 ;  // l grf

//	for ( i = 0 ; i < num_marker_line ; i ++ ) {
	 
//	}

//	printf("instatial_axis=%d\n",instantial_axis) ;

	for ( i = 0 ; i < num_mkr ; i ++ )
	p_mkr[0][i][0] = 0.0 ;
//		printf("sprintf(f_name,.motgc, model_name );は終わったよmain.c\n");	//debug用
	for ( t = 0 ; ; t ++ ) {

		if ( fgets( buffer, 10000, fp ) == NULL ) break ;

	    if ( buffer[0] != '#' ) {
		  real_time[cnt] = atof( strtok( buffer, "," ) ) ;
		    for( i = 0 ; i < 8 ; i++ )
		      seg_angle[cnt][i] = atof( strtok( NULL, "," ) ) ;
		      body_originx[cnt] = atof( strtok( NULL, "," ) ) ;
		      body_originz[cnt] = atof( strtok( NULL, "," ) ) ;

		      p_mkr[cnt][grf_marker_line_s[0]][0] = atof( strtok( NULL, "," ) ) ;
		      p_mkr[cnt][grf_marker_line_s[0]][1] = -0.12 ;
		      p_mkr[cnt][grf_marker_line_s[0]][2] = 0.0 ;
		      p_mkr[cnt][grf_marker_line_f[0]][0] = p_mkr[cnt][grf_marker_line_s[0]][0] + atof( strtok( NULL, "," ) ) / 1000.0 ;
		      p_mkr[cnt][grf_marker_line_f[0]][1] = -0.12 ;
		      p_mkr[cnt][grf_marker_line_f[0]][2] = atof( strtok( NULL, "," ) ) / 1000.0 ;

		      p_mkr[cnt][grf_marker_line_s[1]][0] = atof( strtok( NULL, "," ) ) ;
		      p_mkr[cnt][grf_marker_line_s[1]][1] = 0.12 ;
		      p_mkr[cnt][grf_marker_line_s[1]][2] = 0.0 ;
		      p_mkr[cnt][grf_marker_line_f[1]][0] = p_mkr[cnt][grf_marker_line_s[1]][0] + atof( strtok( NULL, "," ) ) / 1000.0 ;
		      p_mkr[cnt][grf_marker_line_f[1]][1] = 0.12 ;
		      p_mkr[cnt][grf_marker_line_f[1]][2] = atof( strtok( NULL, "," ) ) / 1000.0 ;

//		printf("%d\n", t ) ; // debug用
//		rtoe #5 is origin
			p_mkr[cnt][5][0] = body_originx[cnt] ;
			p_mkr[cnt][5][1] = -0.12 ;
			p_mkr[cnt][5][2] = body_originz[cnt] ;

//		rankle marker #4
			p_mkr[cnt][4][0] = body_originx[cnt] - skeletonLength[4] * cos( seg_angle[cnt][4] / 180.0 * PI) ; // foot segment #4
			p_mkr[cnt][4][1] = -0.12 ;
			p_mkr[cnt][4][2] = body_originx[cnt] + skeletonLength[4] * sin( seg_angle[cnt][4] / 180.0 * PI) ; // foot segment #4
//			printf("%f, %f, %f, %f\n", skeletonLength[4], p_mkr[cnt][4][0], p_mkr[cnt][4][1], p_mkr[cnt][4][2]) ;
//		rfoot segment cog
			seg_cog[cnt][4][0] = body_originx[cnt] - skeletonLength[4] * skeletonCOGr[4] * cos( seg_angle[cnt][4] / 180.0 * PI);
			seg_cog[cnt][4][1] = body_originx[cnt] + skeletonLength[4] * skeletonCOGr[4] * sin( seg_angle[cnt][4] / 180.0 * PI);

//		rknee marker #3
			p_mkr[cnt][3][0] = p_mkr[cnt][4][0] - skeletonLength[3] * cos( seg_angle[cnt][3] / 180.0 * PI) ;// rshank segment #3
			p_mkr[cnt][3][1] = -0.12 ; 
			p_mkr[cnt][3][2] = p_mkr[cnt][4][2] + skeletonLength[3] * sin( seg_angle[cnt][3] / 180.0 * PI) ;
//		rshank segment cog
			seg_cog[cnt][3][0] = p_mkr[cnt][4][0] - skeletonLength[3] * cos( seg_angle[cnt][3] / 180.0 * PI) * skeletonCOGr[3] ;
			seg_cog[cnt][3][1] = p_mkr[cnt][4][2] + skeletonLength[3] * sin( seg_angle[cnt][3] / 180.0 * PI) * skeletonCOGr[3] ;

//		rhip marker #2
			p_mkr[cnt][2][0] = p_mkr[cnt][3][0] - skeletonLength[2] * cos( seg_angle[cnt][2] / 180.0 * PI) ;// rthigh segment #2
			p_mkr[cnt][2][1] = -0.12 ; 
			p_mkr[cnt][2][2] = p_mkr[cnt][3][2] + skeletonLength[2] * sin( seg_angle[cnt][2] / 180.0 * PI) ;
//		rthigh segment cog
			seg_cog[cnt][2][0] = p_mkr[cnt][3][0] - skeletonLength[2] * cos( seg_angle[cnt][2] / 180.0 * PI) * skeletonCOGr[2] ;
			seg_cog[cnt][2][1] = p_mkr[cnt][3][2] + skeletonLength[2] * sin( seg_angle[cnt][2] / 180.0 * PI) * skeletonCOGr[2] ;

//		lumber marker #1
			p_mkr[cnt][1][0] = p_mkr[cnt][2][0] - skeletonLength[1] * cos( seg_angle[cnt][1] / 180.0 * PI) ;// pelvis segment #1
			p_mkr[cnt][1][1] = 0.0 ; 
			p_mkr[cnt][1][2] = p_mkr[cnt][2][2] + skeletonLength[1] * sin( seg_angle[cnt][1] / 180.0 * PI) ;
//		pelvis segment cog
			seg_cog[cnt][1][0] = p_mkr[cnt][2][0] - skeletonLength[1] * cos( seg_angle[cnt][1] / 180.0 * PI) * skeletonCOGr[1] ;
			seg_cog[cnt][1][1] = p_mkr[cnt][2][2] + skeletonLength[1] * sin( seg_angle[cnt][1] / 180.0 * PI) * skeletonCOGr[1] ;

//		head marker #0
			p_mkr[cnt][0][0] = p_mkr[cnt][1][0] - skeletonLength[0] * cos( seg_angle[cnt][0] / 180.0 * PI) ;// HTUL segment #0
			p_mkr[cnt][0][1] = 0.0 ; 
			p_mkr[cnt][0][2] = p_mkr[cnt][1][2] + skeletonLength[0] * sin( seg_angle[cnt][0] / 180.0 * PI) ;
//		HTUL segment cog
			seg_cog[cnt][0][0] = p_mkr[cnt][1][0] - skeletonLength[0] * cos( seg_angle[cnt][0] / 180.0 * PI) * skeletonCOGr[0] ;
			seg_cog[cnt][0][1] = p_mkr[cnt][1][2] + skeletonLength[0] * sin( seg_angle[cnt][0] / 180.0 * PI) * skeletonCOGr[0] ;

//		lhip marker #6
			p_mkr[cnt][6][0] = p_mkr[cnt][2][0] ;
			p_mkr[cnt][6][1] = 0.12 ; 
			p_mkr[cnt][6][2] = p_mkr[cnt][2][2] ;

//		lknee marker #7
			p_mkr[cnt][7][0] = p_mkr[cnt][6][0] + skeletonLength[5] * cos( seg_angle[cnt][5] / 180.0 * PI) ;// lthigh segment #5
			p_mkr[cnt][7][1] = 0.12 ; 
			p_mkr[cnt][7][2] = p_mkr[cnt][6][2] - skeletonLength[5] * sin( seg_angle[cnt][5] / 180.0 * PI) ;
//		lthigh segment cog
			seg_cog[cnt][5][0] = p_mkr[cnt][6][0] + skeletonLength[5] * cos( seg_angle[cnt][5] / 180.0 * PI) * (1.0 - skeletonCOGr[5]) ;
			seg_cog[cnt][5][1] = p_mkr[cnt][6][2] - skeletonLength[5] * sin( seg_angle[cnt][5] / 180.0 * PI) * (1.0 - skeletonCOGr[5]) ;

//		lankle marker #8
			p_mkr[cnt][8][0] = p_mkr[cnt][7][0] + skeletonLength[6] * cos( seg_angle[cnt][6] / 180.0 * PI) ;// lshank segment #6
			p_mkr[cnt][8][1] = 0.12 ; 
			p_mkr[cnt][8][2] = p_mkr[cnt][7][2] - skeletonLength[6] * sin( seg_angle[cnt][6] / 180.0 * PI) ;
//		lshank segment cog
			seg_cog[cnt][6][0] = p_mkr[cnt][7][0] + skeletonLength[6] * cos( seg_angle[cnt][6] / 180.0 * PI) * (1.0 - skeletonCOGr[6]) ;
			seg_cog[cnt][6][1] = p_mkr[cnt][7][2] - skeletonLength[6] * sin( seg_angle[cnt][6] / 180.0 * PI) * (1.0 - skeletonCOGr[6]) ;

//		ltoe marker #9
			p_mkr[cnt][9][0] = p_mkr[cnt][8][0] + skeletonLength[7] * cos( seg_angle[cnt][7] / 180.0 * PI) ;// lfoot segment #7
			p_mkr[cnt][9][1] = 0.12 ; 
			p_mkr[cnt][9][2] = p_mkr[cnt][8][2] - skeletonLength[7] * sin( seg_angle[cnt][7] / 180.0 * PI) ;
//		lfoot segment cog
			seg_cog[cnt][7][0] = p_mkr[cnt][8][0] + skeletonLength[7] * cos( seg_angle[cnt][7] / 180.0 * PI) * (1.0 - skeletonCOGr[7]) ;
			seg_cog[cnt][7][1] = p_mkr[cnt][8][2] - skeletonLength[7] * sin( seg_angle[cnt][7] / 180.0 * PI) * (1.0 - skeletonCOGr[7]) ;
		
			  body_cog[cnt][0] = 0.0 ;
			  body_cog[cnt][1] = 0.0 ;
			for( i = 0 ; i < 8 ; i++ ) {
			  body_cog[cnt][0] += skeletonWeight[i] * seg_cog[cnt][i][0] ;
			  body_cog[cnt][1] += skeletonWeight[i] * seg_cog[cnt][i][1] ;
			}
			  body_cog[cnt][0] /= body_weight ;
			  body_cog[cnt][1] /= body_weight ;
//			  printf( "%f, %f,\n", body_cog[cnt][0], body_weight * GA ) ;
	
			cnt ++;
	    } // buffer != "#"
	}
	fclose(fp); 


}

/* motionデータを読み込む */
void ReadMotdata(void)
{
	FILE *fp;
	char f_name[64],buffer[20000],*fgets();	
	int i,j,k,t,n;	
	double	seg_angle[MAX_LINE][8], body_originx[MAX_LINE], body_originz[MAX_LINE] ;
	double	seg_cog[MAX_LINE][8][2] ;
	int	d ;
	double trx, temp, tempx, tempz ;
	int pelvis_num ;
	int num_objects ;
	double length[2] ;

	cnt = 0 ;


	  sprintf(f_name,"%s.csv", model_name );
	  if ((fp=fopen(f_name,"rt"))==NULL) {
		printf("Cannnot open %s\n",f_name);
		exit(0);
	  }

	//	printf("sprintf(f_name,.motgc, model_name );は終わったよmain.c\n");	//debug用

		if ( fgets( buffer, 10000, fp ) == NULL ) return( 0 ) ;
		  num_objects = atoi( strtok( buffer, "," ) ) ;

		num_mkr = 2 * num_objects ;
		num_ctr = 2 * num_objects ;

		for ( i = 0 ; i < num_ctr ; i ++ ) {
			ctr[i] = i ;
		}

		num_marker_line = num_objects ;

		for ( i = 0 ; i < num_objects ; i ++ ) {
		 marker_line_s[i] = 2*i ; // head
		 marker_line_f[i] = 2*i+1 ;
		}
//		marker_line_s[0] = 0 ; // head
//		marker_line_f[0] = 1 ; 
//		marker_line_s[1] = 2 ; // lumber
//		marker_line_f[1] = 3 ;  // rhip
//		marker_line_s[2] = 4 ; // lumber
//		marker_line_f[2] = 5 ;  // rhip

		if ( fgets( buffer, 10000, fp ) == NULL ) return( 0 ) ;
		  length[0] = atof( strtok( buffer, "," ) ) ;
		 for ( i = 1 ; i < num_objects ; i ++ ) {
		  length[i] = atof( strtok( NULL, "," ) ) ;
		 }
//	for ( i = 0 ; i < num_marker_line ; i ++ ) {
	 
//	}

//	printf("instatial_axis=%d\n",instantial_axis) ;

	for ( i = 0 ; i < num_mkr ; i ++ )
	p_mkr[0][i][0] = 0.0 ;
//		printf("sprintf(f_name,.motgc, model_name );は終わったよmain.c\n");	//debug用
	for ( t = 0 ; ; t ++ ) {

		if ( fgets( buffer, 10000, fp ) == NULL ) break ;

	    if ( buffer[0] != '#' ) {
		  real_time[cnt] = atof( strtok( buffer, "," ) ) ;
		    for( i = 0 ; i < num_objects ; i++ ) {
		      body_originx[cnt] = atof( strtok( NULL, "," ) ) ;
		      body_originz[cnt] = atof( strtok( NULL, "," ) ) ;
		      seg_angle[cnt][i] = atof( strtok( NULL, "," ) ) ;

		      p_mkr[cnt][marker_line_s[i]][0] = body_originx[cnt] + 0.5 * length[i] * cos( seg_angle[cnt][i] ) ;
		      p_mkr[cnt][marker_line_s[i]][1] = 0.0 ;
		      p_mkr[cnt][marker_line_s[i]][2] = body_originz[cnt] + 0.5 * length[i] * sin( seg_angle[cnt][i] ) ;
		      p_mkr[cnt][marker_line_f[i]][0] = body_originx[cnt] - 0.5 * length[i] * cos( seg_angle[cnt][i] ) ;
		      p_mkr[cnt][marker_line_f[i]][1] = 0.0 ;
		      p_mkr[cnt][marker_line_f[i]][2] = body_originz[cnt] - 0.5 * length[i] * sin( seg_angle[cnt][i] ) ;
		    }

	
			cnt ++;
	    } // buffer != "#"
	}
	fclose(fp); 


}

void myReshape(GLsizei width, GLsizei height)
{
	int i;
	GLdouble up[3];
	GLdouble vector[3];
	GLdouble norm;

	glViewport (0,0,width,height);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity();
//	gluPerspective(18.0,(GLfloat)width/(GLfloat)height,5.0,2000.0);
	gluPerspective( 3.0,(GLfloat)width/(GLfloat)height,5.0,2000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	for (i=0;i<3;i++){
		vector[i]=center[i]-eye[i];
	}
	up[0]=-vector[0]*vector[2];
	up[1]=-vector[1]*vector[2];
	up[2]=vector[0]*vector[0]+vector[1]*vector[1];
	norm=up[0]*up[0]+up[1]*up[1]+up[2]*up[2];
	norm=sqrt(norm);
	for (i=0;i<3;i++) up[i] /= norm;
	gluLookAt (eye[0],eye[1],eye[2],center[0],center[1],center[2],
		   up[0],up[1],up[2]);
}




/* メインの表示プログラムとなる */
void display (void) 
{
	int		i,j,m,p, k;
	double	Ang[3], tmp[DIM];
	char	filename[60] ;
	double Vector[3] ;

	extern int	clock() ;
	extern void	drawCylinderShading() ;
	extern double	veclen() ;
	float d = 0.05f, s = 2.0f;
	GLdouble color1[] =  { 0.0, 1.0, 0.0, 0.5} ;

	double v_grf[3], floor_x, floor_z ;
	int ctr_flg ;	// center marker flag
	double grf_vector_length ;


    tmbef = tmnow;
    tm = -99;
    while(tm < tmhz -1){	// リアルタイムタイマー
        tmnow = clock();
        tm = tmnow - tmbef;
    }

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	glTranslatef ( BodyTranslate[0], BodyTranslate[1], BodyTranslate[2]);
	glRotatef ( (GLfloat) ALLROTATE[0], 0.0, 0.0, 1.0 );
	glRotatef ( (GLfloat) ALLROTATE[1],
		    cos((GLfloat)ALLROTATE[0] * PI/180.),
		    -sin((GLfloat)ALLROTATE[0] * PI/180.),0.0);
	
	/* floor draw */
	if ( flr_solid == ON ) { // solid
		glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color4);
	  glBegin(GL_POLYGON);

//		glColor3dv(color1);
//		glNormal3f(0.,0.,1.); 
		floor_x = -9.0 * cos( SLOPE_ANGLE / 180. * PI ) ;
		floor_z = -9.0 * sin( SLOPE_ANGLE / 180. * PI ) ;
		glVertex3f(floor_x,-1.0,-floor_z);
		glVertex3f(floor_x,1.0,-floor_z);
		floor_x = 9.0 * cos( SLOPE_ANGLE / 180. * PI ) ;
		floor_z = 9.0 * sin( SLOPE_ANGLE / 180. * PI ) ;
		glVertex3f(floor_x,1.0,-floor_z);
		glVertex3f(floor_x,-1.0,-floor_z);

	  glEnd();

	} else { // wire flame
		glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color4);
	  glBegin(GL_LINES);

		j = 0 ;
		/* 床が動くのを作る */

		for ( k = 0 ; k < 100 ; k ++ ) {
		   if ( 0.5 * (double)( k ) <= ( qXX[timer] - qXX[0] ) && 0.5 * (double)( k + 1 ) > ( qXX[timer] - qXX[0] ) )
			j = k ;
		}
		for ( i = 0 ; i < 37 ; i ++) {
			floor_x = (-9.0 + 0.5*(double)(i+j) - ( qXX[timer] - qXX[0] )) * cos( SLOPE_ANGLE / 180. * PI ) ;
			floor_z = (-9.0 + 0.5*(double)(i+j) - ( qXX[timer] - qXX[0] )) * sin( SLOPE_ANGLE / 180. * PI ) ;
			glVertex3f(floor_x,-1.0,-floor_z);
			glVertex3f(floor_x, 1.0,-floor_z);
		}

		for ( i = 0 ; i < 5 ; i ++) {
			floor_x = (9.0 - ( qXX[timer] - qXX[0] ) + 0.5 * (double)j) * cos( SLOPE_ANGLE / 180. * PI ) ;
			floor_z = (9.0 - ( qXX[timer] - qXX[0] ) + 0.5 * (double)j) * sin( SLOPE_ANGLE / 180. * PI ) ;
			glVertex3f(floor_x,-1.0+0.5*(double)i,-floor_z);

			floor_x = (-9.0 - ( qXX[timer] - qXX[0] ) + 0.5 * (double)j) * cos( SLOPE_ANGLE / 180. * PI ) ;
			floor_z = (-9.0 - ( qXX[timer] - qXX[0] ) + 0.5 * (double)j) * sin( SLOPE_ANGLE / 180. * PI ) ;
			glVertex3f(floor_x,-1.0+0.5*(double)i,-floor_z);
		}


	  glEnd();
	
	}

//		printf("%d,  %f\n", j, ( qXX[timer] - qXX[0] ) ) ;


///////////// ASCIIデータそのまま入れた場合 マーカの表示
     if ( ASC_flg == ON ) {
//	   glPushMatrix();
      if ( MARKER_flg == ON ) {
	for( i = 0 ; i < num_mkr ; i ++ ) {	
//	  if ( p_mkr[timer][i][ZZ] > p_mkr[timer][i][XX] * sin( - SLOPE_ANGLE / 180. * PI )  ) {
	   glPushMatrix();
		
		glTranslated(p_mkr[timer][i][XX],p_mkr[timer][i][YY],p_mkr[timer][i][ZZ]); // origin of link[i]
//		printf("%d,%f\n",i,tr[timer][i][ZZ]);
		

		for ( j = 0 ; j < num_ctr ; j ++ ) {
		 if ( i == ctr[j] ) {
			ctr_flg = ON ; // 赤にする
		 }
		}

		if ( ctr_flg == ON ) {
		   glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color5 );	// marker色を決める RED
		 } else {
		   glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color3 );	// marker色を決める WHITE
		 }

		glutSolidSphere(0.02, 10, 10) ; 
		ctr_flg = OFF ;

	   glPopMatrix();
//	  } // [ZZ] > 0.00001
	 } // for i
	
       } else { // MARKER_flg
	for( i = 0 ; i < num_mkr ; i ++ ) {
		for ( j = 0 ; j < num_ctr ; j ++ ) {
		 for ( m = 0 ; m < num_marker_line ; m ++ ) {
		  if ( ( i == marker_line_s[m] || i == marker_line_f[m] ) && i == ctr[j] ) {
			ctr_flg = ON ; // 赤にする
		  }
		 }
		}

		if ( ctr_flg == ON ) {
		   glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color5 );	// 色を決める RED


	   glPushMatrix();
		
		glTranslated(p_mkr[timer][i][XX],p_mkr[timer][i][YY],p_mkr[timer][i][ZZ]); // origin of link[i]


		   glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color5 );	// 色を決める RED

		glutSolidSphere(0.02, 10, 10) ; 
	   glPopMatrix();

		 }

		ctr_flg = OFF ;
	 } // i++


       } // // MARKER_flg

      if ( MARKER_LINE_flg == ON ) { // lines between markers
	   glPushMatrix();
	  for ( j = 0 ; j < num_marker_line ; j ++ ) {  //　オブジェクトの番号
	   if( j % 2 == 0 ) // %:余剰を計算する演算子
             glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color3 ); // 色を決めるwhite
	   if( j % 2 == 1 )
             glMaterialfv(GL_FRONT,GL_DIFFUSE,material_color2 ); // 色を決めるgreen
		drawCylinderShading( p_mkr[timer][marker_line_s[j]][XX],p_mkr[timer][marker_line_s[j]][YY],p_mkr[timer][marker_line_s[j]][ZZ],
		 p_mkr[timer][marker_line_f[j]][XX],p_mkr[timer][marker_line_f[j]][YY],p_mkr[timer][marker_line_f[j]][ZZ], 0.015, 0); // 円柱を書く命令

	  } 

		




	   glPopMatrix();
         } //MARKER_LINE_flg == ON
	} // ASC_flg


//		axis(d,s);
 //		glFlush ();
	glutSwapBuffers();

//	glPopMatrix();


	glPopMatrix();

//	auxSwapBuffers();

	if ( eps_flg == 1 ) {
		sprintf( filename, "%03d.eps", timer ) ;
		generateEPS( filename,  1, WIDTH2, HEIGHT2 ); 
	}
}




void axis(float d, float s)
{
    glPushMatrix();
    subaxis(d,s);
    glPopMatrix();

    glPushMatrix();
    glRotatef(90.0, 1.0, 0.0, 0.0);
    subaxis(d,s);
    glPopMatrix();

    glPushMatrix();
    glRotatef(-90.0, 0.0, 0.0, 1.0);
    subaxis(d,s);
    glPopMatrix();
}

void subaxis(float d, float s)
{
    glTranslatef(0.0, s - 1.0, 0.0);
//    auxSolidCylinder(d, 2.0*s);
    glTranslatef(0.0, 1.0, 0.0);
    glRotatef(-90.0, 1.0, 0.0, 0.0);
//    auxSolidCone(2.0*d, 4.0*d);
    return;
}




/* 光源設定 */
void myinit (void)
{
	GLfloat light0_position[]={-5.0,0.0,5.0,0.0};
	GLfloat light1_position[]={5.0,-5.0,0.0,0.0};
	GLfloat light2_position[]={0.0,5.0,0.0,0.0};
	GLfloat light3_position[]={2.5,0.0,-5.0,0.0};

	GLfloat light0_diffuse[]={0.8,0.8,0.8,1.0};
	GLfloat light1_diffuse[]={0.8,0.8,0.8,1.0};
	GLfloat light2_diffuse[]={0.8,0.8,0.8,1.0};
	GLfloat light3_diffuse[]={0.8,0.8,0.8,1.0};

//	GLfloat light_specular[]={0.3,0.3,0.3,1.0};
	GLfloat light_specular[]={1.,1.,1.0,1.0};

	GLfloat lmodel_ambient[]={0.9,0.9,0.9,1.0};

	glLightfv(GL_LIGHT0,GL_POSITION,light0_position);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,light0_diffuse);
	glLightfv(GL_LIGHT0,GL_SPECULAR,light_specular);

	glLightfv(GL_LIGHT1,GL_POSITION,light1_position);
	glLightfv(GL_LIGHT1,GL_DIFFUSE,light1_diffuse);
	glLightfv(GL_LIGHT1,GL_SPECULAR,light_specular);

	glLightfv(GL_LIGHT2,GL_POSITION,light2_position);
	glLightfv(GL_LIGHT2,GL_DIFFUSE,light2_diffuse);
	glLightfv(GL_LIGHT2,GL_SPECULAR,light_specular);

	glLightfv(GL_LIGHT3,GL_POSITION,light3_position);
	glLightfv(GL_LIGHT3,GL_DIFFUSE,light3_diffuse);
	glLightfv(GL_LIGHT3,GL_SPECULAR,light_specular);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,lmodel_ambient);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);

	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POINT_SMOOTH);
}


/* マウスの設定 */
// using examples from http://atlas.riken.jp/~koishi/claret.html
void mouse(int button, int state, int x, int y)
{
  switch (button) {
  case GLUT_LEFT_BUTTON:
    if (state == GLUT_DOWN) {
      mpos[0] = x;
      mpos[1] = y;
      mouse_l = 1;
    }
    if (state == GLUT_UP) {
      mouse_l = 0;
    }
//	printf("%d,%d,%d left \n", x, y, mouse_l ) ;		// debug用
    break;
/*
  case GLUT_MIDDLE_BUTTON:
    if (state == GLUT_DOWN) {
      mpos[0] = x;
      mpos[1] = y;
      mouse_m = 1;
    }
    if (state == GLUT_UP) {
      mouse_m = 0;
    }
	printf("%d,%d,%d middle \n", x, y, mouse_m ) ;		// debug用
    break;
*/
  case GLUT_RIGHT_BUTTON:
    if (state == GLUT_DOWN) {
      mpos[0] = x;
      mpos[1] = y;
      mouse_r = 1;
    }
    if (state == GLUT_UP) {
      mouse_r = 0;
    }
//	printf("%d,%d,%d right \n", x, y, mouse_r ) ;		// debug用
    break;
  default:
    break;
  }
}

/* マウスの動作を反映して，視点を切り替える */
// using examples from http://atlas.riken.jp/~koishi/claret.html
void motion(int x, int y)
{
  double d0;
  double len = 10.0;
/*
  len = eye_len;

  if(mouse_l == 1 && mouse_m == 1){
    trans[0] -= (double)(y-mpos[1])*len/150;
    angle[0] = -(double)(x-mpos[0])*0.2;
  } else  if(mouse_m == 1 || (mouse_l == 1 && mouse_r == 1)){
    trans[1] -= (double)(x-mpos[0])*len*.001;
    trans[2] += (double)(y-mpos[1])*len*.001;
  } else if(mouse_r == 1){
    trans[0] += (double)(y-mpos[1])*len/150;
    angle[0] =  (double)(x-mpos[0])*0.2;
  } else if(mouse_l == 1){
    d0 = len/50;
    if(d0 > 1.0) d0 = 1.0;
    angle[1] = (double)(y-mpos[1])*d0;
    angle[2] = (double)(x-mpos[0])*d0;
  }
  if(mouse_l == 1 || mouse_m == 1 || mouse_r == 1){
    mpos[0] = x;
    mpos[1] = y;
    glutPostRedisplay();
  }
*/

	if( mouse_l == 1 && mouse_r == 1 ){ // translation
//	    trans[1] -= (double)(x-mpos[0])*len*.0001;
//	    trans[2] += (double)(y-mpos[1])*len*.0001;
	    BodyTranslate[0] += (double)(x-mpos[0])*len*.001;
	    BodyTranslate[2] -= (double)(y-mpos[1])*len*.001;
	    glutPostRedisplay() ;


	} else if( mouse_r == 1 ){ // zoom in/out
//	    trans[0] += (double)(y-mpos[1])*len/150 + (double)(x-mpos[0])*len/150 ;
//	    angle[0] =  (double)(x-mpos[0])*0.2;
	    BodyTranslate[1] += (double)(y-mpos[1])*len/150 + (double)(x-mpos[0])*len/150 ;
	    glutPostRedisplay();
	} else if( mouse_l == 1 ){ // angle change aroud Y?/Z
	    d0 = len/50;
	    if(d0 > 1.0) d0 = 1.0;
	    angle[1] = (double)(y-mpos[1])*d0;
	    angle[2] = (double)(x-mpos[0])*d0;
//	printf("%f,%d,%d\n", angle[1],ALLROTATE[0],ALLROTATE[1] ) ;	// debug用
	    ALLROTATE[0] = (ALLROTATE[0] + (int)angle[2] ) % 360;
	    ALLROTATE[1] = (ALLROTATE[1] + (int)angle[1] ) % 360;
	    glutPostRedisplay();
//	printf("%f,%d,%d\n", angle[1],ALLROTATE[0],ALLROTATE[1] ) ;	// debug用
	}
	if(mouse_l == 1 || mouse_m == 1 || mouse_r == 1){
	    mpos[0] = x;
	    mpos[1] = y;
	    glutPostRedisplay() ;
//	printf("%f,%f,%d\n", mpos[0],mpos[1],ALLROTATE[1] ) ;	// debug用
	}



}


void Count_Func (void)
{
	if ( Stop && timer < cnt-1-1 ) {
		timer++ ; 
		display();
//		glutPostRedisplay();
	}
	if ( timer == cnt-1-1 ) {
		if (loop_flg == OFF ) Stop = 0;
		timer = 0;
	}
}


GLvoid *grabPixels(int inColor, unsigned int width, unsigned int height)
{
  GLvoid *buffer;
  GLint swapbytes, lsbfirst, rowlength;
  GLint skiprows, skippixels, alignment;
  GLenum format;
  unsigned int size;

  if (inColor) {
    format = GL_RGB;
    size = width * height * 3;
  } else {
    format = GL_LUMINANCE;
    size = width * height * 1;
  }

  buffer = (GLvoid *) malloc(size);
  if (buffer == NULL)
    return NULL;

  /* Save current modes. */
  glGetIntegerv(GL_PACK_SWAP_BYTES, &swapbytes);
  glGetIntegerv(GL_PACK_LSB_FIRST, &lsbfirst);
  glGetIntegerv(GL_PACK_ROW_LENGTH, &rowlength);
  glGetIntegerv(GL_PACK_SKIP_ROWS, &skiprows);
  glGetIntegerv(GL_PACK_SKIP_PIXELS, &skippixels);
  glGetIntegerv(GL_PACK_ALIGNMENT, &alignment);
  /* Little endian machines (DEC Alpha for example) could
     benefit from setting GL_PACK_LSB_FIRST to GL_TRUE
     instead of GL_FALSE, but this would require changing the
     generated bitmaps too. */
  glPixelStorei(GL_PACK_SWAP_BYTES, GL_FALSE);
  glPixelStorei(GL_PACK_LSB_FIRST, GL_FALSE);
  glPixelStorei(GL_PACK_ROW_LENGTH, 0);
  glPixelStorei(GL_PACK_SKIP_ROWS, 0);
  glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  /* Actually read the pixels. */
  glReadPixels(0, 0, width, height, format,
    GL_UNSIGNED_BYTE, (GLvoid *) buffer);

  /* Restore saved modes. */
  glPixelStorei(GL_PACK_SWAP_BYTES, swapbytes);
  glPixelStorei(GL_PACK_LSB_FIRST, lsbfirst);
  glPixelStorei(GL_PACK_ROW_LENGTH, rowlength);
  glPixelStorei(GL_PACK_SKIP_ROWS, skiprows);
  glPixelStorei(GL_PACK_SKIP_PIXELS, skippixels);
  glPixelStorei(GL_PACK_ALIGNMENT, alignment);
  return buffer;
}


int generateEPS(char *filename, int inColor, unsigned int width, unsigned int height)
{
  FILE *fp;
  GLvoid *pixels;
  unsigned char *curpix;
  int components, pos, i;

  pixels = grabPixels(inColor, width, height);
  if (pixels == NULL)
    return 1;
  if (inColor)
    components = 3;     /* Red, green, blue. */
  else
    components = 1;     /* Luminance. */

  fp = fopen(filename, "w");
  if (fp == NULL) {
    return 2;
  }
  fprintf(fp, "%%!PS-Adobe-2.0 EPSF-1.2\n");
  fprintf(fp, "%%%%Creator: OpenGL pixmap render output\n");
  fprintf(fp, "%%%%BoundingBox: 0 0 %d %d\n", width, height);
  fprintf(fp, "%%%%EndComments\n");
  fprintf(fp, "gsave\n");
  fprintf(fp, "/bwproc {\n");
  fprintf(fp, "    rgbproc\n");
  fprintf(fp, "    dup length 3 idiv string 0 3 0\n");
  fprintf(fp, "    5 -1 roll {\n");
  fprintf(fp, "    add 2 1 roll 1 sub dup 0 eq\n");
  fprintf(fp, "    { pop 3 idiv 3 -1 roll dup 4 -1 roll dup\n");
  fprintf(fp, "        3 1 roll 5 -1 roll put 1 add 3 0 }\n");
  fprintf(fp, "    { 2 1 roll } ifelse\n");
  fprintf(fp, "    } forall\n");
  fprintf(fp, "    pop pop pop\n");
  fprintf(fp, "} def\n");
  fprintf(fp, "systemdict /colorimage known not {\n");
  fprintf(fp, "    /colorimage {\n");
  fprintf(fp, "        pop\n");
  fprintf(fp, "        pop\n");
  fprintf(fp, "        /rgbproc exch def\n");
  fprintf(fp, "        { bwproc } image\n");
  fprintf(fp, "    } def\n");
  fprintf(fp, "} if\n");
  fprintf(fp, "/picstr %d string def\n", width * components);
  fprintf(fp, "%d %d scale\n", width, height);
  fprintf(fp, "%d %d %d\n", width, height, 8);
  fprintf(fp, "[%d 0 0 %d 0 0]\n", width, height);
  fprintf(fp, "{currentfile picstr readhexstring pop}\n");
  fprintf(fp, "false %d\n", components);
  fprintf(fp, "colorimage\n");

  curpix = (unsigned char *) pixels;
  pos = 0;
  for (i = width * height * components; i > 0; i--) {
    fprintf(fp, "%02hx", *curpix++);
    if (++pos >= 32) {
      fprintf(fp, "\n");
      pos = 0;
    }
  }
  if (pos)
    fprintf(fp, "\n");

  fprintf(fp, "grestore\n");
  free(pixels);
  fclose(fp);
  return 0;
}




void keyboard1(unsigned char key, int x, int y)
{
	char	filename[60] ;

	switch (key) {
		case 'q':
			exit(0);
		case 'Q':
			exit(0);
		case 'z':
			ALLROTATE[0] = (ALLROTATE[0] + 2) % 360 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 'Z':
			ALLROTATE[0] = (ALLROTATE[0] - 2) % 360 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 'x':
			ALLROTATE[1] = (ALLROTATE[1] + 2) % 360 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 'X':
			ALLROTATE[1] = (ALLROTATE[1] - 2) % 360 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 'i':
			BodyTranslate[1] -= 0.1;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 'o':
			BodyTranslate[1] += 0.1;
			if (Stop == 0) glutPostRedisplay();
			break;
		case 's':
			if (Stop) Stop = 0;	// Stop == 1 ---> Stop = 0
			else Stop = 1; // Stop == 0 ---> Stop = 1
			break;
		case 'f':
			if ( timer >= cnt-1-1 )	timer = 0 ;
			if ( timer < cnt-1-1 )	timer++ ;
			glutPostRedisplay();
// 			printf ("%d", timer ) ;			// debug用
			break;
		case 'b':
			if ( timer <= 0 ) timer = cnt-1-1;
			if ( timer > 0 )	timer--;
			glutPostRedisplay();
			break;
		case 'p':
			sprintf( filename, "%03d.eps", timer ) ;
			generateEPS( filename,  1, WIDTH2, HEIGHT2 );
			break;
		case '\033':  /* '\033' は ESC の ASCII コード */
			exit(0);
		case 'm':
			if ( mscl_flg == ON ) mscl_flg = OFF ;
			else mscl_flg = ON ;
			glutPostRedisplay();
			break;
		case 'g':
			if ( grf_flg == ON ) grf_flg = OFF ;
			else grf_flg = ON ;
			glutPostRedisplay();
			break;
		case 'c':
			if ( cog_flg == ON ) cog_flg = OFF ;
			else cog_flg = ON ;
			glutPostRedisplay();
			break;
		case 'j':
			if ( jo_ball_flg == ON ) jo_ball_flg = OFF ;
			else jo_ball_flg = ON ;
			glutPostRedisplay();
			break;
		case 'a':
			if ( ASC_flg == OFF && ASC_read_flg == ON ) ASC_flg = ON ;
			else ASC_flg = OFF ;
			glutPostRedisplay();
			break;

		case 'A':
			if ( MARKER_flg == OFF ) MARKER_flg = ON ;
			else MARKER_flg = OFF ;
			glutPostRedisplay();
			break;
		case 'L':
			if ( MARKER_LINE_flg == OFF ) MARKER_LINE_flg = ON ;
			else MARKER_LINE_flg = OFF ;
			glutPostRedisplay();
			break;

		case 'n':
			if ( stick_flg == OFF && stick_read_flg == ON ) stick_flg = ON ;
			else stick_flg = OFF ;
			glutPostRedisplay();
			break;

		case 'w':
			if ( flr_solid == ON ) flr_solid = OFF ;
			else flr_solid = ON ;
			glutPostRedisplay();
			break;
		case '-':
			tmhz += 5 ;
			glutPostRedisplay();
			printf( "%d \n", tmhz ) ;
			break;
		case '+':
			tmhz -= 5 ;
			glutPostRedisplay();
			printf( "%d \n", tmhz ) ;
			break;
		default:
			break;

	}
}


void keyboard2(int key, int x, int y)
{

	switch (key) {

		case GLUT_KEY_LEFT:
			BodyTranslate[0] += 0.01 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case GLUT_KEY_RIGHT:
			BodyTranslate[0] -= 0.01;
			if (Stop == 0) glutPostRedisplay();
			break;
		case GLUT_KEY_UP:
			BodyTranslate[2] += 0.01 ;
			if (Stop == 0) glutPostRedisplay();
			break;
		case GLUT_KEY_DOWN:
			BodyTranslate[2] -= 0.01;
			if (Stop == 0) glutPostRedisplay();
			break;
		default:
			break;

	}
}



int main (int argc,char *argv[])
{
	int		i ;

	if(argc <= 1) {
		printf("USAGE: wvs1 -fFileName -mIntervalTime(msec)\n\n");
		exit(-1);
	}
	for (i=1; i<argc; i++) {
		if (argv[i][0] == '-') {
			switch (argv[i][1]) {
				case 'f':	strcpy(model_name, &(argv[i][2])); // file name
							break;
				case 'm':	sscanf( argv[i], "-m%d", &tmhz ); // sampling rate
							break;
				case 't':	trans_flg = OFF ;
							break ;
				case 'l':	loop_flg = ON ;
							Stop = 1; // Stop という名のフラグ Stop = 1のとき連続して動作
							break ;
				case 'p':	eps_flg = 1 ;
							break;
				case 'n':	if ( argv[i][2] == 's' )
							stick_flg = OFF ;
							break ;

				case 'c':	data_gc_flg = 1 ; // motgcというファイルを読み込む
						grf_flg = ON ;
						cog_flg = ON ;
							break;
				case 'k':	mscl_flg = OFF ;
							break;
				case 'w':	flr_solid = OFF ;
							break ;
				case 'A':	//sscanf( argv[i], "-A%d", &num_mkr ); // number_of_markers
						ASC_flg = ON ;
							break ;

				case 'P':	POS_flg = ON ;
							break ;

				case 'B':	INV_FRONT_flg = ON ;
							break ;

				default:	printf("USAGE: animation -fFileName -mIntervalTime(msec) [-t] [-p] [-A] [-w] [-l] [-B] \n\n");
							exit(-1);
							break;
			}
				loop_flg = ON ;
				Stop = 1; // Stop という名のフラグ Stop = 1のとき連続して動作
				ASC_flg = ON ;
				tmhz = 8 ;
						
		} 
		else { 
			printf("USAGE: animation -fFileName -mIntervalTime(msec) [-t] [-p] [-A] [-w] [-l] [-B] \n\n");
			exit(-1);
		}
	}

//		printf("read motionの前だよmain.c\n");			// debug用

//		printf("read motionは終わったよmain.c\n");		// debug用
//		ReadSkeleton() ; // read skeleton data
//		ReadAngle() ; // read angle data
		ReadMotdata() ; // read angle data
		ASC_read_flg = ON ;




	glutInit(&argc, argv);
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH ); // ダブルバッファリングとデプスバッファの使用
	glutInitWindowPosition(WX, WY);
  	glutInitWindowSize(WIDTH2, HEIGHT2);
 	glutCreateWindow(argv[0]);
	myinit ();

//	glPushMatrix();
//	glTranslated(0.0,0.0,3.0);
//	YsDrawUglyFont("(0,0,3)",1);
//	glPopMatrix();


	glutMouseFunc(mouse);
  	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard1);
	glutSpecialFunc(keyboard2);


	glutReshapeFunc(myReshape);
	glutIdleFunc( Count_Func );
	glutDisplayFunc(display) ;
 	glutMainLoop();
 	return 0;

}


