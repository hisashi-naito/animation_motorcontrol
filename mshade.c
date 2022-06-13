/*
  �C�ӂ̕������������~���f�[�^�̐���
*/
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/***** iida add 2000/8 *****/
#pragma warning(disable : 4244)     // MIPS
#pragma warning(disable : 4136)     // X86
#pragma warning(disable : 4051)     // ALPHA

#define drawOneLine(x1,y1,z1,x2,y2,z2) glBegin(GL_LINES); \
 glVertex3d((x1),(y1),(z1)); glVertex3d((x2),(y2),(z2)); glEnd();

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

void tran( double x, double y, double z, double m[4][4] ) ;
void rotz( double s, double c, double m[4][4] ) ;
void concatinate( double a[4][4], double b[4][4] );
void copymx(double a[4][4], double b[4][4] ) ;
void unitmx( double v[4][4] ) ;
int unitvec( double u[] ) ;
double veclen( double u[3] ) ;    /* �x�N�g���̒��������߂� */


/*****:::::::::::::::::*****/
	
void VRMLinit(FILE *fpout){
    fprintf(fpout,"#VRML V2.0 utf8\n");
    fprintf(fpout,"Viewpoint { position 0 0 20 }\n");
}

void VRMLshvx(FILE *fpout){
    fprintf(fpout,"Shape { appearance Appearance { material Material { \n");
    fprintf(fpout,"                  diffuseColor 0.7 1 0.2 } }\n");
    fprintf(fpout," geometry IndexedFaceSet { coord Coordinate { point [ \n");
}

void VRMLvxeg(FILE *fpout){ fprintf(fpout,"   ] }  coordIndex [ \n"); } 

void VRMLegend(FILE *fpout){ fprintf(fpout," ] creaseAngle 1.5 } }\n"); }

double veclen( u )    /* �x�N�g���̒��������߂� */
double u[];
{
      return( sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]) );
}
 
double innerproduct( double a[], double b[] )  /* �x�N�g���̓��ς����߂� */
{    return( a[0]*b[0]+a[1]*b[1]+a[2]*b[2] ); }

void vecpro( double a[], double b[], double c[] )
/*   c = a X b �@�x�N�g���̊O�ς����߂�  */
{
      c[0] = a[1]*b[2] - a[2]*b[1];
      c[1] = a[2]*b[0] - a[0]*b[2];
      c[2] = a[0]*b[1] - a[1]*b[0];
}

void direcmat( double z[], double m[][4] ) 
{         /* �{���̂����������A�����̂������� ��]����s��𐶐��@*/
      double x[3],y[3],u[]={1.,0.,0.}, v[]={0.,1.,0.};
      double a[4][4];
      int i;


      unitvec( z ); // unit�x�N�g����
			// zXX, zYY, zZZ�͉~����
      if( fabs(innerproduct(z,u)) < fabs(innerproduct(z,v)) ){ //u:x���̒P�ʃx�N�g���^v:y���̒P�ʃx�N�g��
								// z�̕���������΍��W��y���ɋ߂�������ix�������j
          vecpro( z, u, y ) ; // y: �P�ʈʒu�x�N�g����x���Ƃ̖@���x�N�g��
	  unitvec( y ) ; // 
	  vecpro( y, z, x ) ; // ���̖@���x�N�g���ƒP�ʈʒu�x�N�g���̖@���x�N�g��
	  unitvec( x ) ; // 
			// x, y, z �Ǐ����W���̐�΍��W�ɑ΂���e�����x�N�g��
      }
      else { vecpro(v,z,x); unitvec(x); vecpro(z,x,y); unitvec(y); }
      for( i=0; i<3; i++ ) {
          a[0][i]=x[i]; a[1][i]=y[i]; a[2][i]=z[i]; a[3][i]=0.; a[i][3]=0.;
      }
	      a[3][3]=1.;
/*
		xXX, xYY, xZZ, 0
		yXX, yYY, yZZ, 0
		zXX, zYY, zZZ, 0
		  0,   0,   0, 1
*/

      concatinate( a,m ); // m = a X m

/* m
		xXX, xYY, xZZ, 0
		yXX, yYY, yZZ, 0
		zXX, zYY, zZZ, 0
		CXX, CYY, CZZ, 1
�����rotz�ɂ�����ƂR�C�S��͕ω�����
*/




}              


void drawCylinderShading (double tx,double ty,double tz,
                        double bx,double by,double bz , double radius, int cone_flg )
                        /*GLfloat radius, GLint division, GLfloat DD)*/
{
#include "color.h"
        /////char filename[30]="aaa.iv";
        /////int division=20; /* �~���̕����� */
        /////float radius=10.; /* �~���̔��a */

 double   ln[2][3]; /* �~���̒�ʂ̒��S���W */

//FILE *fp;
double ss,cc,s1,c1,axis[3],center[3],m[4][4],sd;
double angle=0.,height;
int i, j,ii;

    int iv,iw0,iw1,iw2,iw3;

//        GLfloat radius;
        GLint division;
	GLfloat  xx,yy,zz;
        double pv[24][3];
	GLfloat vertex_data[4][3];
	GLfloat normal[3];
	double norm;
	GLfloat vec0[3],vec1[3];

//        radius = df;	// color.h�Œ�`

	division = div;

//		radius = rr ;

        /////printf("radius=%f   division=%d    DD=%f  \n",df,div,DD);

        ln[0][0] = tx; ln[0][1] = ty; ln[0][2] = tz; // �N�n�_
        ln[1][0] = bx; ln[1][1] = by; ln[1][2] = bz; // ��~�_

        /////printf("Output Inventor File = %s\n",filename);

        /////if( (fp=fopen(filename,"w"))<= 0 ){
        /////    printf("File %s cannot be created \n",filename);
        /////    exit(0);
        /////}

        /////VRMLinit(fp);
        /////VRMLshvx(fp);

        /////fp = fopen(filename,"w");

        /////fprintf(fp,"#Inventor V2.0 ascii\n");
        /////fprintf(fp,"Separator {\n");
        /////fprintf(fp,"#HS         graphicRendering ON\n");
        /////fprintf(fp,"#HS         hapticRendering ON\n");
        /////fprintf(fp,"    Coordinate3 {\n");
        /////fprintf(fp,"    #HS         graphicRendering OFF\n");
        /////fprintf(fp,"    #HS         hapticRendering OFF\n");
        /////fprintf(fp,"        point [\n");

        /* �~�����[�̒�ʂ̒��S���W��o�^ */
        ///fprintf(fp,"                %10.4f %10.4f %10.4f,\n",
        ///                            ln[1][0],ln[1][1],ln[1][2]);
        ///fprintf(fp,"                %10.4f %10.4f %10.4f,\n",
        ///                            ln[0][0],ln[0][1],ln[0][2]);
        axis[0] = ln[1][0] - ln[0][0];    /* �~���������x�N�g�� */
        axis[1] = ln[1][1] - ln[0][1];
        axis[2] = ln[1][2] - ln[0][2];
        center[0] = (ln[0][0] + ln[1][0])/2.; /* �~���̒��S���W�l */
        center[1] = (ln[0][1] + ln[1][1])/2.;
        center[2] = (ln[0][2] + ln[1][2])/2.;
        height = veclen( axis )/2.;       /* �~���̍����̔��� */
        sd = (double)division;            /* �~���̕����� */
        ii = -1; // to line 175 

        for( i=0; i<division; i++ ){

          angle = 2.*3.141592654*(double)i/sd; 
          s1 = radius*sin( angle ); c1 = radius*cos( angle );

          unitmx( m ); // 4x4�̒P�ʍs��̍쐬
          tran( center[0],center[1],center[2],m ); //�~���̒��S�_�ʒu�s��̍쐬
          direcmat( axis,m );      /* �~���̃��[�J�����W�{�ʒu�s��̍쐬 */   /////////////////////////// 
          tran( 0.,0.,height,m );  /* �~���̏��ʂ֕��s�ړ��ϊ� */
          rotz( s1,c1,m );         /* �~���̒��S���܂���m����]�ϊ�+�g��radius�� */

          /* �~����Ő���̂P�_���L�^ */
          /////fprintf(fp,"                %10.4f %10.4f %10.4f,\n",
          /////        m[0][0]+m[3][0], m[0][1]+m[3][1], m[0][2]+m[3][2]);
          ++ii; // �܂�0, 2 ,4

	if ( cone_flg == 0 ) {
          pv[ii][0] = m[0][0]+m[3][0]; // ���[�J��x�������x�N�g���̐����{�~���̏��ʂ̒��Sx���W
          pv[ii][1] = m[0][1]+m[3][1]; // ���[�J��y�������x�N�g���̐����{�~���̏��ʂ̒��Sy���W
          pv[ii][2] = m[0][2]+m[3][2]; // ���[�J��z�������x�N�g���̐����{�~���̏��ʂ̒��Sz���W
	} else {
          pv[ii][0] = m[3][0]; // ���[�J��x�������x�N�g���̐����{�~���̏��ʂ̒��Sx���W
          pv[ii][1] = m[3][1]; // ���[�J��y�������x�N�g���̐����{�~���̏��ʂ̒��Sy���W
          pv[ii][2] = m[3][2]; // ���[�J��z�������x�N�g���̐����{�~���̏��ʂ̒��Sz���W
 	}

//          unitmx( m ); // 4x4�̒P�ʍs��̍쐬
//          tran( center[0],center[1],center[2],m ); //�~���̒��S�_�ʒu�s��̍쐬
//          direcmat( axis,m );      /* �~���̃��[�J�����W�{�ʒu�s��̍쐬 */  ///////////////////////////////
          tran( 0.,0.,-2. * height,m ); /* �~���̉���ʂ֕��s�ړ��ϊ� */
//          rotz( s1,c1,m );         /* �~���̒��S���܂���m����]�ϊ�+�g��?*/

          /* �~�����Ő���̂P�_���L�^ */
          /////fprintf(fp,"                %10.4f %10.4f %10.4f,\n",
          /////        m[0][0]+m[3][0], m[0][1]+m[3][1], m[0][2]+m[3][2]);
          ++ii; // 1,3
          pv[ii][0] = m[0][0]+m[3][0]; // ���[�J��x�������x�N�g���̐����{�~���̉���ʂ̒��Sx���W
          pv[ii][1] = m[0][1]+m[3][1]; // ���[�J��y�������x�N�g���̐����{�~���̉���ʂ̒��Sy���W
          pv[ii][2] = m[0][2]+m[3][2]; // ���[�J��z�������x�N�g���̐����{�~���̉���ʂ̒��Sz���W
        }

        /////VRMLvxeg(fp);
        /////fprintf(fp,"       ]\n");
        /////fprintf(fp,"    }\n");
        /////fprintf(fp,"    IndexedFaceSet {\n");
        /////fprintf(fp,"    #HS         graphicRendering ON\n");
        /////fprintf(fp,"    #HS         hapticRendering ON\n");
        /////fprintf(fp,"    #HS         pointNormal OFF\n");
        /////fprintf(fp,"              coordIndex [\n");

        /* surface */
        /*        surface(fp,division);   */
        /////loop(fp,division);

    for( iv=0; iv<division; iv++ ){
       iw0 = iv*2;
       iw1 = iv*2 +1;
       iw2 = iv*2 +2;
       iw3 = iv*2 +3;
       if(iw2>=(division*2))   iw2=iw2-(division*2);
       if(iw3>=(division*2+1)) iw3=iw3-(division*2);
       /////fprintf(fp,"                %d, %d, %d, %d, -1,\n",iw3,iw2,iw0,iw1); 

		/////for (i=0;i<4;++i) {
		/////	m = order[j][i];
		/////	for(n=0; n<3; n++) vertex_data[i][n] = pv[m][n];
		/////}
/*****
        vertex_data[0][0]=(float)pv[iw3][0]; 
	vertex_data[0][1]=(float)pv[iw3][1]; 
	vertex_data[0][2]=(float)pv[iw3][2]; 

        vertex_data[1][0]=(float)pv[iw2][0]; 
	vertex_data[1][1]=(float)pv[iw2][1]; 
	vertex_data[1][2]=(float)pv[iw2][2]; 

        vertex_data[2][0]=(float)pv[iw0][0]; 
	vertex_data[2][1]=(float)pv[iw0][1]; 
	vertex_data[2][2]=(float)pv[iw0][2]; 

        vertex_data[3][0]=(float)pv[iw1][0]; 
	vertex_data[3][1]=(float)pv[iw1][1]; 
	vertex_data[3][2]=(float)pv[iw1][2]; 
*****/

        vertex_data[0][0]=(float)pv[iw1][0]; 
	vertex_data[0][1]=(float)pv[iw1][1]; 
	vertex_data[0][2]=(float)pv[iw1][2]; 

        vertex_data[1][0]=(float)pv[iw0][0]; 
	vertex_data[1][1]=(float)pv[iw0][1]; 
	vertex_data[1][2]=(float)pv[iw0][2]; 

        vertex_data[2][0]=(float)pv[iw2][0]; 
	vertex_data[2][1]=(float)pv[iw2][1]; 
	vertex_data[2][2]=(float)pv[iw2][2]; 

        vertex_data[3][0]=(float)pv[iw3][0]; 
	vertex_data[3][1]=(float)pv[iw3][1]; 
	vertex_data[3][2]=(float)pv[iw3][2]; 

		for (i=0;i<3;++i) {
			vec0[i]=vertex_data[3][i]-vertex_data[1][i];
			vec1[i]=vertex_data[0][i]-vertex_data[1][i];
		}

		normal[0]=vec0[1]*vec1[2]-vec0[2]*vec1[1];
		normal[1]=vec0[2]*vec1[0]-vec0[0]*vec1[2];
		normal[2]=vec0[0]*vec1[1]-vec0[1]*vec1[0];

		norm=normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2];
		norm = sqrt(norm);
		for (i=0;i<3;++i) normal[i] /= norm;


		/* GL_POLYGON�̕`�� */
		glBegin(GL_POLYGON);
			glNormal3f(normal[0],normal[1],normal[2]); 
			for (i=0;i<4;i++){
			glVertex3f(vertex_data[i][0],
				   vertex_data[i][1],
				   vertex_data[i][2]); // 4�̒��_
			}
		glEnd();

    }

        /////fprintf(fp,"     ]\n");
        /////fprintf(fp,"    }\n");
        /////fprintf(fp,"}\n");

        /////VRMLegend(fp);
        /////fclose( fp );
}

printUsage(void)
{
  printf("USAGE: mebius [-fInventorFileName] ");
  printf( "[-dDivision] [-rRadius] [-wWidth]\n\n");
  exit(-1);
}

void unitmx( v )
	double v[4][4];
{
	int i, j;
	for( i=0; i<4; i++ ) {
    	 for( j=0; j<4; j++ ) v[i][j]=0.;
	}
        	v[0][0]=v[1][1]=v[2][2]=v[3][3]=1.;
}

int unitvec( u )
double u[];
{
      double len;

      len = veclen( u );
      if( fabs(len) < 1.e-8 ) {
           printf("Err:   Vector length is zero / %f / %f / %f / %f /\n",
                   len, u[0], u[1], u[2] );
           u[0]=1.; u[1]=0.; u[2]=0.; return 1;
      }
      u[0] /= len; u[1] /= len; u[2] /= len;
      return 0;
}

/* 4x4�s�񓯎m�̍s��� c = a X b */
void multmx( a,b,c )
	double a[4][4],b[4][4],c[4][4];
{	
	int i,j;

	for( i=0; i<4; i++ ) {
    		for( j=0; j<4; j++ )
		c[i][j]=a[i][0]*b[0][j]+a[i][1]*b[1][j]+a[i][2]*b[2][j]
			+ a[i][3]*b[3][j];
	}
}

void tran( x,y,z,m )
double x,y,z,m[4][4];
{
double a[4][4];

a[0][0]=1.; a[0][1]=0.; a[0][2]=0.; a[0][3]=0.;
a[1][0]=0.; a[1][1]=1.; a[1][2]=0.; a[1][3]=0.;
a[2][0]=0.; a[2][1]=0.; a[2][2]=1.; a[2][3]=0.;
a[3][0]=x;  a[3][1]=y;  a[3][2]=z;  a[3][3]=1.;
concatinate( a,m );
}

void rotx( s,c,m )
double s,c,m[4][4];
{
double a[4][4];

a[0][0]=1.; a[0][1]=0.; a[0][2]=0.; a[0][3]=0.;
a[1][0]=0.; a[1][1]=c;  a[1][2]=s;  a[1][3]=0.;
a[2][0]=0.; a[2][1]=-s; a[2][2]=c;  a[2][3]=0.;
a[3][0]=0.; a[3][1]=0.; a[3][2]=0.; a[3][3]=1.;
concatinate( a,m );
}

void roty( s,c,m )
double s,c,m[4][4];
{
double a[4][4];

a[0][0]=c;  a[0][1]=0.; a[0][2]=-s; a[0][3]=0.;
a[1][0]=0.; a[1][1]=1.; a[1][2]=0.; a[1][3]=0.;
a[2][0]=s;  a[2][1]=0.; a[2][2]=c;  a[2][3]=0.;
a[3][0]=0.; a[3][1]=0.; a[3][2]=0.; a[3][3]=1.;
concatinate( a,m );
}

void rotz( s,c,m )
double s,c,m[4][4];
{
double a[4][4];

a[0][0]=c;  a[0][1]=s;  a[0][2]=0.; a[0][3]=0.;
a[1][0]=-s; a[1][1]=c;  a[1][2]=0.; a[1][3]=0.;
a[2][0]=0.; a[2][1]=0.; a[2][2]=1.;  a[2][3]=0.;
a[3][0]=0.; a[3][1]=0.; a[3][2]=0.; a[3][3]=1.;
concatinate( a,m );
}


void scale( x,y,z,m )
double x,y,z,m[4][4];
{
double a[4][4];

a[0][0]=x;  a[0][1]=0.; a[0][2]=0.; a[0][3]=0.;
a[1][0]=0.; a[1][1]=y;  a[1][2]=0.; a[1][3]=0.;
a[2][0]=0.; a[2][1]=0.; a[2][2]=z;  a[2][3]=0.;
a[3][0]=0.; a[3][1]=0.; a[3][2]=0.; a[3][3]=1.;
concatinate( a,m );
}

/* �s��a�ƍs��b�̐ς�b�ŕԂ� */
void concatinate( a,b )
double a[4][4],b[4][4];
{
double z[4][4];

multmx( a,b,z ); // z = a X b
copymx( z,b ); // b = z ���
}

/* �s��a��b�ɑ�� */
void copymx( a,b )
double a[4][4],b[4][4];
{
b[0][0]=a[0][0]; b[0][1]=a[0][1]; b[0][2]=a[0][2]; b[0][3]=a[0][3];
b[1][0]=a[1][0]; b[1][1]=a[1][1]; b[1][2]=a[1][2]; b[1][3]=a[1][3];
b[2][0]=a[2][0]; b[2][1]=a[2][1]; b[2][2]=a[2][2]; b[2][3]=a[2][3];
b[3][0]=a[3][0]; b[3][1]=a[3][1]; b[3][2]=a[3][2]; b[3][3]=a[3][3];
}

void writeline( fp,v,k,m )
FILE *fp;
double v[][3];
int k;
double m[][4];
{
double x1,y1,z1,x2,y2,z2;

k *= 2;
x1 = v[0][0]*m[0][0]+v[0][1]*m[1][0]+v[0][2]*m[2][0]+m[3][0];
y1 = v[0][0]*m[0][1]+v[0][1]*m[1][1]+v[0][2]*m[2][1]+m[3][1];
z1 = v[0][0]*m[0][2]+v[0][1]*m[1][2]+v[0][2]*m[2][2]+m[3][2];
x2 = v[1][0]*m[0][0]+v[1][1]*m[1][0]+v[1][2]*m[2][0]+m[3][0];
y2 = v[1][0]*m[0][1]+v[1][1]*m[1][1]+v[1][2]*m[2][1]+m[3][1];
z2 = v[1][0]*m[0][2]+v[1][1]*m[1][2]+v[1][2]*m[2][2]+m[3][2];
fprintf(fp,"                %10.4f %10.4f %10.4f,\n",x1,y1,z1);
fprintf(fp,"                %10.4f %10.4f %10.4f,\n",x2,y2,z2);
}

void loop( FILE *fp, int n)
{
    int i,j;
    int iv,iw0,iw1,iw2,iw3;

    for( iv=0; iv<n; iv++ ){

       iw0 = iv*2;
       iw1 = iv*2 +1;
       iw2 = iv*2 +2;
       iw3 = iv*2 +3;

       if(iw2>=(n*2))   iw2=iw2-(n*2);
       if(iw3>=(n*2+1)) iw3=iw3-(n*2);
       fprintf(fp,"                %d, %d, %d, %d, -1,\n",iw3,iw2,iw0,iw1); 

    }


}

void surface( fp , division)
FILE *fp;
int division;
{
int i,iv,iw0,iw1,iw2;
    for(iv=0; iv<division; iv++){
    for(i=0; i<2; i++){
      if(i==0){
       iw0 = iv*2;
       iw1 = iv*2 +1;
       iw2 = iv*2 +2;
       if(iw2>=(division*2)) iw2=iw2-(division*2-1); 
       fprintf(fp,"                %d, %d, %d, -1,\n",iw0,iw1,iw2);
      }
      else{
       iw0 = iv*2 +1;
       iw1 = iv*2 +3;
       if(iw1>=(division*2+1)) iw1=iw1-(division*2+1); 
       iw2 = iv*2 +2;
       if(iw2>=(division*2)) iw2=iw2-(division*2-1); 
       fprintf(fp,"                %d, %d, %d, -1,\n",iw0,iw1,iw2); 
      }
    }
    }
}
