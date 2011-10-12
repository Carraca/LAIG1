#include <gl\glui.h>
#include <math.h>
#include "RGBpixmap.h"


// quadricas
GLUquadric* glQ;	// nec. p/ criar sup. quadraticas (cilindros, esferas...)


// dimensoes e localizacao da janela
#define DIMX 500
#define DIMY 500
#define INITIALPOS_X 200
#define INITIALPOS_Y 200

#define DP_GEOMETRY 1
#define DP_MACHINE 2
#define DP_ROBOT 3

#define TX_NEWSPAPER 1
#define TX_LANDSCAPE 2
#define TX_WOOD_BELT 3
#define TX_WOOD_MACHINE 4
#define TX_METAL 5
#define TX_BELT 6
#define TX_FLOOR 7
#define TX_GUILLOTINE 8
#define TX_WALL 9

const int TRUE  = 1;
const int FALSE = 0;

float xy_aspect;
float width,height;

double pi=atan(1.0) * 4.0;
double two_pi=2*pi;
double half_pi=pi/2.0;

// matriz de transf. geometrica utilizada pelo botao esferico
float view_rotate[16] = { 1,0,0,0,
                          0,1,0,0,
                          0,0,1,0,
                          0,0,0,1 };

// vector de posicao utilizado pelo botao de afastamento
float obj_pos[] = { 0.0, 0.0, 0.0 };

// declarações para os tres eixos (cilindros)
double axis_radius_begin =  0.4;
double axis_radius_end   =  0.0;
double axis_lenght       = 32.0;
int axis_nslices = 8;
int axis_nstacks = 1;

float mat1_shininess[] = {128.0}; 
float mat1_specular[] = {0.3, 0.3, 0.3, 1.0};
float mat1_diffuse[] =  {0.7, 0.7, 0.7, 1.0};	
float mat1_ambient[] =  {0.7, 0.7, 0.7, 1.0};

float matWhite_shininess[] = {128.0}; 
float matWhite_specular[] = {0.3, 0.3, 0.3, 1.0};
float matWhite_diffuse[] =  {1.0, 1.0, 1.0, 1.0};
float matWhite_ambient[] =  {1.0, 1.0, 1.0, 1.0};

float matMetal_shininess[] = {76.8}; 
float matMetal_specular[] = {.574, .574, .574, 1.0};
float matMetal_diffuse[] =  {.5, .5, .5, 1.0};
float matMetal_ambient[] =  {.45, .45, .45, 1.0};

float roomX = 40;
float roomY = 45;
float roomZ = 70;

float impostorX0 = -70;
float impostorX1 = 90;
float impostorY0 = -50;
float impostorY1 = 90;
float impostorZ0 = -90;
float impostorZ1 = 90;

float windowYi = 10;
float windowYf = 25;
float windowZi = -35;
float windowZf = 35;

float doorXi = 10;
float doorXf = 20;
float doorY = 20;

int curvedNSlices = 40;
int curvedYSlices = 20;
float curvedXHeight = 10;

float roomXMax = roomX+curvedXHeight;

//declaracoes para o chao (evaluators)
int floorXSlices = 100;
int floorZSlices = roomZ/(roomXMax/((double) floorXSlices));

GLfloat floorCtrlPoints[4][3] = {{  -roomX, 0.0, roomZ},
								{  -roomX, 0.0, -roomZ}, 
								{ roomXMax, 0.0, roomZ},
								{ roomXMax, 0.0, -roomZ} };

GLfloat floorNrmlCompon[4][3] = {{  0.0, 1.0, 0.0},
								{  0.0, 1.0, 0.0}, 
								{  0.0, 1.0, 0.0},
								{  0.0, 1.0, 0.0} };

GLfloat floorColorPoints[4][4] = {{ 0.0, 0.2, 0.2, 0},
								{ 0.0, 0.0, 0.2, 0}, 
								{ 0.0, 0.2, 0.0, 0},
								{ 0.2, 0.0, 0.0, 0} };

int txRepetition = 10;
float floorTxX = (roomX+roomXMax)/(2*roomZ);

GLfloat floorTextPoints[4][2] = {{ 0.0, 0.0},
								{ 0.0, txRepetition}, 
								{ txRepetition*floorTxX, 0.0},
								{ txRepetition*floorTxX, txRepetition} };

float machineX = 7.5;//metade
float machineY = 8;
float machineZ = 5;//metade
float machineXaux = 4;//5;
float machineYaux = 6;

float conveyorX = machineXaux+1;
float conveyorY = machineYaux-2*machineXaux/pi;//2;
float conveyorZ = 16;
float beltTxcoordZ=conveyorZ/conveyorX;

float teapotSize = 2.5;

float printingMachineX=-20;
float printingMachineZ=-25;

float guillotineSupportSide = 0.25;
float guillotineSupportHeight = 6;
float guillotineSupportX = conveyorX - guillotineSupportSide/2.0;
float guillotineSupportY = conveyorY;
float guillotineSupportZ = machineZ + conveyorZ;

float guillotineX = conveyorX-guillotineSupportSide/2.0;
float guillotineY = 0.75;
float guillotinePosX = printingMachineX;
float guillotinePosYi= conveyorY+guillotineY;
float guillotinePosYf= conveyorY+guillotineSupportHeight-guillotineY;
float guillotinePosZ = printingMachineZ + machineZ + conveyorZ;
float guillotineTxX = guillotineY/guillotineX;

float platenRadius = (machineYaux-conveyorY)/2.0;
float platenHeight = (2*machineXaux);
float platenSlices = 20;

//coordenadas do centro do circulo de tras
float platenX = printingMachineX + machineXaux;
float platenY = conveyorY + platenRadius;
float platenZ = printingMachineZ + machineZ;

float robotBodyX = 6;
float robotBodyY = 2.5;
float robotBodyZ = 6;

double robotClipPlane[]={0,1,0,0};
double robotLightRadius = 0.5;
double robotLightOuterRadius = 1.5;
double robotLightHeight=10;

//coordenadas em relação ao centro do robot
double robotLightX = robotBodyX - 1;
double robotLightZ = -robotBodyZ + 1;

double robotWheelRadius = 1;
double robotWHeelSlices = 20;
double robotWheelHeight = 0.25;
double robotWheelX= robotBodyX - robotWheelRadius - 0.5;

//coordenada do centro do "chao" do robot
double robotX = -20;
double robotY = 1;
double robotZ = 20;

// declarações para a fonte de luz LIGHT0;
float light0_position[]  = {0.0, 3.0, 4.0, 1.0}; 
float light0_ambient[] =   {0.0, 0.0, 0.0, 1.0};
float light0_diffuse[] =   {25.0, 25.0, 25.0, 1.0};
float light0_specular[] =  {5.0, 5.0, 5.0, 1.0};
float light0_kc = 0.0;
float light0_kl = 1.0;
float light0_kq = 0.0;
double light0x = 0;
double light0y = 10;
double light0z = 0;
double symb_light0_radius = 0.2;
int symb_light0_slices = 8;
int symb_light0_stacks =8;

//FOCO DE LUZ
// declarações para a fonte de luz LIGHT1;
float light1_position[]  = {0.0, 3.0, 4.0, 1.0}; 
float light1_ambient[] =   {0.0, 0.0, 0.0, 1.0};
float light1_diffuse[] =   {2.0, 2.0, 2.0, 1.0};
float light1_specular[] =  {2.0, 2.0, 2.0, 1.0};
float light1_spot_direction[] = {1,-3,0};
float light1_cone_radius = 30;
float light1_kc = 0.0;
float light1_kl = 1.0;
float light1_kq = 0.0;
double light1x = robotLightX + 2*robotLightOuterRadius;
double light1y = robotY+ robotLightHeight;
double light1z = robotLightZ;
double symb_light1_radius = 0.2;
int symb_light1_slices = 8;
int symb_light1_stacks =8;
int light1_active=TRUE;

double newspapersX = conveyorX-1;
double newspapersY = conveyorY+0.1;
double newspapersZi = -conveyorZ;
double newspapersZf=newspapersZi;
double newspapersTxInc=0;

double newspaperTxInc=0;
double newspaperZi=conveyorZ;
double newspaperZf=newspaperZi;
double newspaperZF=conveyorZ+2*machineXaux;
double newspaperX = printingMachineX;
double newspaperZ = printingMachineZ+conveyorZ+machineZ;

double paperYi = newspapersY;
double paperZi = newspaperZf;
double paperYf = robotY+.1;
double paperZf = robotZ;

//animaçoes
//ST0 corresponde ao estado em que a maquina esta a trabalhar, os restantes ao movimento do robot
//ST00 inicio da animação, vai ate a folha inicial chegar ao final do tapete
//ST01 uma folha sai fora do tapete
//ST02 a folha cai no tapete
enum State{ST00,ST01,ST02,ST1,ST2,ST3,ST4,ST5,ST6};
State st=ST00;
double delta_t=10;
double v = 20.0;
double trajectoryRadius= 10;
double delta_x = v*(delta_t/1000.0);
double delta_z = delta_x;
double delta_ang = v/trajectoryRadius*180.0/pi*delta_t/1000.0;
double robotXi = robotX;
double robotXf = (doorXi+doorXf)/2.0;
double robotZi = robotZ;
double robotZf = -roomZ+robotBodyZ;
double centerX = robotXf-trajectoryRadius;
double centerZ = robotZi-trajectoryRadius;
double ang=0;
double ang_radians=0;
double robotXa=robotX;
double robotZa=robotZ;
double txInc=0.0;
double txinc=0.01;
double belt_delta_z =2*machineXaux*txinc;
double platen_delta_ang= belt_delta_z/platenRadius*180.0/pi;
double platen_ang=0;

double paperZa=paperZi;
double paperYa=paperYi;

double guillotineYa=guillotinePosYi;

int newspapersTotal=4;
int n_newspapers=0;

double guillotineInc= ((guillotinePosYf-guillotinePosYi)/((2*machineXaux)/belt_delta_z))*2.0;
int guillotineUp=TRUE;

//CÂMARAS
int cam=1;

double cam2PosX = 0;
double cam2PosY = 200;
double cam2PosZ = 0;
double cam2RefX = 0;
double cam2RefY = 0;
double cam2RefZ = 0;
double cam2VectX = -1;
double cam2VectY = 0;
double cam2VectZ = 0;

double cam3PosX = 0;
double cam3PosY = 20;
double cam3PosZ = 0;
double cam3RefX = robotXa;
double cam3RefY = robotY + (robotLightHeight+robotLightOuterRadius)/2.0;
double cam3RefZ = robotZa;
double cam3VectX = 0;
double cam3VectY = 1;
double cam3VectZ = 0;

//botoes
GLUI_Translation *trans_z;
GLUI_Rotation *view_rot;

// fonte (global) de luz ambiente 
float light_ambient[] = {0.6, 0.6, 0.6, 1.0}; /* Set the background ambient lighting. */

// variaveis para a janela
int main_window;
GLUI  *glui2;

RGBpixmap pixmap;

///////////////////////////////////////////////////////////
//Sala

void myCircle(double radius, int slices, double z_normal){
	double inc=two_pi/((double) slices);
	double ang= 0.0;

	if(z_normal<0)
		glFrontFace(GL_CW);

	glPushMatrix();
	glScaled(radius,radius,1);
	glBegin(GL_POLYGON);
		glNormal3d(0.0,0.0,z_normal);
		while(ang<two_pi){
			glVertex3d(cos(ang),sin(ang),0.0);
			ang+=inc;
		}
	glEnd();
	glPopMatrix();

	if(z_normal<0)
		glFrontFace(GL_CCW);
}

void myCylinder(double radius, double height, int slices, int bases)
{
	double d_angle = 2*pi/slices;
	double sin_d_angle=sin(d_angle);
	double cos_d_angle=cos(d_angle);
	double d_angle_degrees = d_angle*180/pi;
	double angle = 0;
	double px,py;
	px=radius*cos(d_angle); 
	py=radius*sin(d_angle);
	double t_texture_inc= 1.0/((double) slices);
	
	int i;

	for(i=0;i<slices;++i){

		glPushMatrix();
		glRotated(angle,0.0,0.0,1); 
		glBegin(GL_POLYGON);
			glNormal3d(1,0.0,0.0);
			glVertex3d( radius,  0.0,  0.0);
			glNormal3d(cos_d_angle,sin_d_angle,0.0);
			glVertex3d( px,py  ,  0.0);
			glVertex3d( px,py ,  height);
			glNormal3d(1,0.0,0.0);
			glVertex3d( radius,0 ,  height);
		glEnd();
		glPopMatrix();

		angle+=d_angle_degrees;
	}
	
	if(bases){
		glPushMatrix();
		glTranslated(0,0,height);
		myCircle(radius,slices,1.0);
		glPopMatrix();

		myCircle(radius,slices,-1.0);
	}
}

void myCylinderTexture(double radius, double height, int slices, int bases, int texture)
{
	double d_angle = 2*pi/slices;
	double sin_d_angle=sin(d_angle);
	double cos_d_angle=cos(d_angle);
	double d_angle_degrees = d_angle*180/pi;
	double angle = 0;
	double px,py;
	px=radius*cos(d_angle); 
	py=radius*sin(d_angle);
	double t_texture_inc= 1.0/((double) slices);
	int i;

	glEnable(GL_TEXTURE_2D);
	for(i=0;i<slices;++i){

		glPushMatrix();
		glRotated(angle,0.0,0.0,1); 
		glBindTexture(GL_TEXTURE_2D, texture);
		glBegin(GL_POLYGON);
			glNormal3d(1,0.0,0.0);
			glTexCoord2f(1.0,-i*t_texture_inc);glVertex3d( radius,  0.0,  0.0);
			glNormal3d(cos_d_angle,sin_d_angle,0.0);
			glTexCoord2f(1.0,-(i+1)*t_texture_inc);glVertex3d( px,py  ,  0.0);
			glTexCoord2f(0.0,-(i+1)*t_texture_inc);glVertex3d( px,py ,  height);
			glNormal3d(1,0.0,0.0);
			glTexCoord2f(0.0,-i*t_texture_inc);glVertex3d( radius,0 ,  height);
		glEnd();
		glPopMatrix();

		angle+=d_angle_degrees;
	}
	glDisable(GL_TEXTURE_2D);

	if(bases){
		glPushMatrix();
		glTranslated(0,0,height);
		myCircle(radius,slices,1.0);
		glPopMatrix();

		myCircle(radius,slices,-1.0);
	}
}

void impostors(){
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat1_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  mat1_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   mat1_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   mat1_ambient);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, TX_LANDSCAPE);

	glBegin(GL_POLYGON);
	glNormal3d(0,0,1);
	glTexCoord2f(0.0,0.0); glVertex3d(impostorX0,impostorY0,impostorZ0);
	glTexCoord2f(1.0,0.0); glVertex3d(impostorX1,impostorY0,impostorZ0);
	glTexCoord2f(1.0,(impostorY1-impostorY0)/(impostorX1-impostorX0)); glVertex3d(impostorX1,impostorY1,impostorZ0);
	glTexCoord2f(0.0,(impostorY1-impostorY0)/(impostorX1-impostorX0)); glVertex3d(impostorX0,impostorY1,impostorZ0);
	glEnd();

	glBegin(GL_POLYGON);
	glNormal3d(1,0,0);
	glTexCoord2f(0.0,0.0); glVertex3d(impostorX0,impostorY0,impostorZ0);
	glTexCoord2f(0.0,(impostorY1-impostorY0)/(impostorX1-impostorX0)); glVertex3d(impostorX0,impostorY1,impostorZ0);
	glTexCoord2f(1.0,(impostorY1-impostorY0)/(impostorX1-impostorX0)); glVertex3d(impostorX0,impostorY1,impostorZ1);
	glTexCoord2f(1.0,0.0); glVertex3d(impostorX0,impostorY0,impostorZ1);
	glEnd();

	glDisable(GL_TEXTURE_2D);
}

void unit_curved_wall(int n_slices,int y_slices){
	double inc=two_pi/((double) n_slices);
	double zi=-pi;
	double xi=cos(zi);
	double zf,xf,sinzi,sinzf;
	sinzi=sin(zi);
	
	double yi,yf;
	double y_inc=1.0/((double) y_slices);

	int r=5;
		double txY= r*roomY/(2*roomX);
		double txYinc= txY/((double) y_slices);
		double txXinc= txYinc*inc/y_inc/2;
		double tx_xi=0;
		double tx_xf=tx_xi+txXinc;
		double tx_yi=0;
		double tx_yf=0;

	do{
		zf=zi+inc;
		xf=cos(zf);
		sinzf=sin(zf);
		if(sinzf<0) sinzf=-sinzf;
		
		yi=yf=0;
		tx_yi=tx_yf=0;
		tx_xf=tx_xi+txXinc;

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D,TX_WALL);
		for(int i=0;i<y_slices;++i){
			yf+=y_inc;
			tx_yf+=txYinc;

			glBegin(GL_POLYGON);
				glNormal3d(-1,0,sinzi);
				glTexCoord2f(tx_xi,tx_yf); glVertex3d(xi,yf,zi);
				glTexCoord2f(tx_xi,tx_yi); glVertex3d(xi,yi,zi);
				glNormal3d(-1,0,sinzf);
				glTexCoord2f(tx_xf,tx_yi); glVertex3d(xf,yi,zf);
				glTexCoord2f(tx_xf,tx_yf); glVertex3d(xf,yf,zf);
			glEnd();

			glBegin(GL_POLYGON);
				glNormal3d(-1,0,-sinzf);
				glTexCoord2f(-tx_xf,tx_yf); glVertex3d(xf,yf,-zf);
				glTexCoord2f(-tx_xf,tx_yi); glVertex3d(xf,yi,-zf);
				glNormal3d(-1,0,-sinzi);
				glTexCoord2f(-tx_xi,tx_yi); glVertex3d(xi,yi,-zi );
				glTexCoord2f(-tx_xi,tx_yf); glVertex3d(xi,yf,-zi);
			glEnd();

			yi=yf;
			tx_yi=tx_yf;
		}
		glDisable(GL_TEXTURE_2D);
		tx_xi=tx_xf;

		xi=xf;
		zi=zf;
		sinzi=sinzf;
	}while(xf<1);
}

void curvedWall(double xi,double zi, double zf,double y_height, double x_height,int n_slices,int y_slices){
	glEnable(GL_NORMALIZE);
	glPushMatrix();
	glTranslated(xi,0,zi);
	glScaled(x_height/2.0,y_height,(zf-zi)/(2*pi));
	glTranslated(1.0,0,pi);//x [-1,1] -> x[0,2]
	unit_curved_wall(n_slices,y_slices);
	glPopMatrix();
	glDisable(GL_NORMALIZE);
}

void windowWall(double x, double yi, double yf, double zi, double zf, double w_yi, double w_yf, double w_zi, double w_zf){
    int r=5;
	double txY= r*roomY/(2*roomX);
	double txWyi= txY*w_yi/(yf-yi);
	double txWyf= txY*w_yf/(yf-yi);
	double txX= r*roomZ/roomX;
	double txWxf= txX*(zf - w_zf)/(zf-zi);
	double txWxi= txX*(zf-w_zi)/(zf-zi);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,TX_WALL);
	//baixo
    glBegin(GL_POLYGON);
        glNormal3d(1.0,0.0,0.0);
		glTexCoord2f(txWxi,0); glVertex3d(x,yi,w_zi);
        glTexCoord2f(txWxi,txWyi); glVertex3d(x,w_yi,w_zi);
        glTexCoord2f(txWxf,txWyi); glVertex3d(x,w_yi,w_zf);
        glTexCoord2f(txWxf,0); glVertex3d(x,yi,w_zf);
    glEnd();

    //cima
    glBegin(GL_POLYGON);
        glNormal3d(1.0,0.0,0.0);
        glTexCoord2f(txWxi,txWyf); glVertex3d(x,w_yf,w_zi);
        glTexCoord2f(txWxi,txY); glVertex3d(x,yf,w_zi);
        glTexCoord2f(txWxf,txY); glVertex3d(x,yf,w_zf);
        glTexCoord2f(txWxf,txWyf); glVertex3d(x,w_yf,w_zf);
    glEnd();

    //direita centro
    glBegin(GL_POLYGON);
        glNormal3d(1.0,0.0,0.0);
        glTexCoord2f(txX,txWyi); glVertex3d(x,w_yi,zi);
        glTexCoord2f(txX,txWyf); glVertex3d(x,w_yf,zi);
        glTexCoord2f(txWxi,txWyf); glVertex3d(x,w_yf,w_zi);
        glTexCoord2f(txWxi,txWyi); glVertex3d(x,w_yi,w_zi);
    glEnd();

	//direita cima
    glBegin(GL_POLYGON);
        glNormal3d(1.0,0.0,0.0);
        glTexCoord2f(txX,txWyf); glVertex3d(x,w_yf,zi);
        glTexCoord2f(txX,txY); glVertex3d(x,yf,zi);
        glTexCoord2f(txWxi,txY); glVertex3d(x,yf,w_zi);
        glTexCoord2f(txWxi,txWyf); glVertex3d(x,w_yf,w_zi);
    glEnd();

	//direita baixo
    glBegin(GL_POLYGON);
        glNormal3d(1.0,0.0,0.0);
        glTexCoord2f(txX,0); glVertex3d(x,yi,zi);
        glTexCoord2f(txX,txWyi); glVertex3d(x,w_yi,zi);
		glTexCoord2f(txWxi,txWyi); glVertex3d(x,w_yi,w_zi);
        glTexCoord2f(txWxi,0); glVertex3d(x,yi,w_zi);
    glEnd();
    
    //esquerda centro
    glBegin(GL_POLYGON);
        glNormal3d(1.0,0.0,0.0);
        glTexCoord2f(txWxf,txWyi); glVertex3d(x,w_yi,w_zf);
        glTexCoord2f(txWxf,txWyf); glVertex3d(x,w_yf,w_zf);
        glTexCoord2f(0,txWyf); glVertex3d(x,w_yf,zf);
        glTexCoord2f(0,txWyi); glVertex3d(x,w_yi,zf);
    glEnd();

	//esquerda cima
    glBegin(GL_POLYGON);
        glNormal3d(1.0,0.0,0.0);
        glTexCoord2f(txWxf,txWyf); glVertex3d(x,w_yf,w_zf);
		glTexCoord2f(txWxf,txY); glVertex3d(x,yf,w_zf);
        glTexCoord2f(0,txY); glVertex3d(x,yf,zf);
        glTexCoord2f(0,txWyf); glVertex3d(x,w_yf,zf);
    glEnd();

	//esquerda baixo
    glBegin(GL_POLYGON);
        glNormal3d(1.0,0.0,0.0);
        glTexCoord2f(txWxf,0); glVertex3d(x,yi,w_zf);
		glTexCoord2f(txWxf,txWyi); glVertex3d(x,w_yi,w_zf);
        glTexCoord2f(0,txWyi); glVertex3d(x,w_yi,zf);
        glTexCoord2f(0,0); glVertex3d(x,yi,zf);
    glEnd();

	glDisable(GL_TEXTURE_2D);
}

void doorWall(double xi, double xf, double yi, double yf, double d_xi, double d_xf, double d_y, double z){
    int r=5;
	double txY= r*roomY/(2*roomX);
	double txDy= txY*d_y/(yf-yi);
	double txDxf= r*(d_xf-xi)/(xf-xi);
	double txDxi= r*(d_xi-xi)/(xf-xi);
	
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,TX_WALL);
	//cima esquerda
    glBegin(GL_POLYGON);
        glNormal3d(0.0,0.0,1.0);
		glTexCoord2f(0,txDy); glVertex3d(xi,d_y,z);
		glTexCoord2f(txDxi,txDy); glVertex3d(d_xi,d_y,z);
        glTexCoord2f(txDxi,txY); glVertex3d(d_xi,yf,z);
        glTexCoord2f(0,txY); glVertex3d(xi,yf,z);
    glEnd();

	//cima centro
	glBegin(GL_POLYGON);
        glNormal3d(0.0,0.0,1.0);
        glTexCoord2f(txDxi,txDy); glVertex3d(d_xi,d_y,z);
        glTexCoord2f(txDxf,txDy); glVertex3d(d_xf,d_y,z);
        glTexCoord2f(txDxf,txY); glVertex3d(d_xf,yf,z);
        glTexCoord2f(txDxi,txY); glVertex3d(d_xi,yf,z);
    glEnd();

	//cima direita
	glBegin(GL_POLYGON);
        glNormal3d(0.0,0.0,1.0);
        glTexCoord2f(txDxf,txDy); glVertex3d(d_xf,d_y,z);
        glTexCoord2f(r,txDy); glVertex3d(xf,d_y,z);
        glTexCoord2f(r,txY); glVertex3d(xf,yf,z);
        glTexCoord2f(txDxf,txY); glVertex3d(d_xf,yf,z);
    glEnd();

    //baixo esquerda
    glBegin(GL_POLYGON);
        glNormal3d(0.0,0.0,1.0);
        glTexCoord2f(0,0); glVertex3d(xi,yi,z);
        glTexCoord2f(txDxi,0); glVertex3d(d_xi,yi,z);
        glTexCoord2f(txDxi,txDy); glVertex3d(d_xi,d_y,z);
        glTexCoord2f(0,txDy); glVertex3d(xi,d_y,z);
    glEnd();
    
    //baixo direita
    glBegin(GL_POLYGON);
        glNormal3d(0.0,0.0,1.0);
        glTexCoord2f(txDxf,0); glVertex3d(d_xf,yi,z);
        glTexCoord2f(r,0); glVertex3d(xf,yi,z);
        glTexCoord2f(r,txDy); glVertex3d(xf,d_y,z);
        glTexCoord2f(txDxf,txDy); glVertex3d(d_xf,d_y,z);
    glEnd();

	glDisable(GL_TEXTURE_2D);
}

void floor(){
	// INICIALIZACOES RELACIONADAS COM OS "EVALUATORS"

	// declaram-se quatro interpoladores, de coordenadas, de
	//     normais, de cores e de texturas:
	// o parâmetro de controlo dos interpoladors varia de 0 a 1,
	//     tanto em U como em V
	// os strides (ordem de visita no array de dados final) são:
	//     3 e 6 para o interpol. de coord. (respectivamente U e V)
	//     3 e 6 para o interpol. de normais (respectivamente U e V)
	//     4 e 8 para o interpolador de cores (respectivamente U e V)
	//     2 e 4 para o interpolador de texturas (respectivamente U e V)
	// a interpolação é linear (de grau 1) pelo que se coloca o
	//     valor "2" (grau + 1) nos quatro casos
	glMap2f(GL_MAP2_VERTEX_3, 0.0, 1.0, 3, 2,  0.0, 1.0, 6, 2,  &floorCtrlPoints[0][0]);
	glMap2f(GL_MAP2_NORMAL,   0.0, 1.0, 3, 2,  0.0, 1.0, 6, 2,  &floorNrmlCompon[0][0]);
	glMap2f(GL_MAP2_COLOR_4,  0.0, 1.0, 4, 2,  0.0, 1.0, 8, 2,  &floorColorPoints[0][0]);
	glMap2f(GL_MAP2_TEXTURE_COORD_2,  0.0, 1.0, 2, 2,  0.0, 1.0, 4, 2,  &floorTextPoints[0][0]);

	// os interpoladores activam-se:
	glEnable(GL_MAP2_VERTEX_3);
	glEnable(GL_MAP2_NORMAL);
	glEnable(GL_MAP2_COLOR_4);
	glEnable(GL_MAP2_TEXTURE_COORD_2);

	// para este conjunto de interpoladores:
	//    na direccao U, serao efectuadas divisoes em 40 passos
	//        quando a variável U varia de 0 a 1
	//    na direccao V, serao efectuadas divisoes em 60 passos
	//        quando a variável U varia de 0 a 1
	glMapGrid2f(floorZSlices, 0.0,1.0,floorXSlices, 0.0,1.0); 

	// SEGUE-SE EXEMPLO DE UTILIZACAO DE "EVALUATORS"
	glShadeModel(GL_SMOOTH);					// GL_FLAT, GL_SMOOTH
	glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, TX_FLOOR);
	glEvalMesh2(GL_FILL, 0,floorZSlices, 0,floorXSlices);		// GL_POINT, GL_LINE, GL_FILL
	//glEvalMesh2(GL_FILL, 10,30, 20,40);	// poligono incompleto...
	//glEvalMesh2(GL_FILL, -10,50, -20,70);	// ...ou "transbordante"
	// NOTA: os 4 ultimos parametros da funcao glEvalMesh2() nao sao 
	//		 coordenadas mas sim indices de passos (do passo -10
	//		 ao passo 50; do passo -20 ao passo 70...

	// so' para referencia visual... onde estao os quatro pontos
	// de controlo:
	glDisable(GL_TEXTURE_2D);
}

void myRoom(double x, double y, double z, double w_yi, double w_yf, double w_zi, double w_zf, double d_xi, double d_xf, double d_y, double x_height,int n_slices,int y_slices){
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat1_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  mat1_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   mat1_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   mat1_ambient);
	
	curvedWall(x,-z,z,y,x_height,n_slices,y_slices);
	doorWall(-x,x,0,y,d_xi,d_xf,d_y,-z);
	windowWall(-x,0,y,-z,z,w_yi,w_yf,w_zi,w_zf);

	//parede frente
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,TX_WALL);

	int r= 5;
	double txY= r*roomY/(2*roomX);
	glBegin(GL_POLYGON);
		glNormal3d(0.0,0.0,-1.0);
		glTexCoord2f(0,0); glVertex3d(x,0,z);
		glTexCoord2f(r,0); glVertex3d(-x,0,z);
		glTexCoord2f(r,txY); glVertex3d(-x,y,z);
		glTexCoord2f(0,txY); glVertex3d(x,y,z);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	//tecto
	glBegin(GL_POLYGON);
		glNormal3d(0.0,-1.0,0.0);
		glVertex3d(x+x_height,y,z);
		glVertex3d(-x,y,z);
		glVertex3d(-x,y,-z);
		glVertex3d(x+x_height,y,-z);
	glEnd();

	floor();
}
////////////////////////////////////////////////////////////
//MAQUINA

//x e z -> metade
void machine(double x, double y, double z, double x_aux , double y_aux){
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, TX_WOOD_MACHINE);

	float txCoordZ= x/z;
	float txCoordY= y/(2*x);
	float txCoordYaux= y_aux/(2*x);
	float txCoordXaux= (x-x_aux)/(2*x);

	//topo
	glBegin(GL_POLYGON);
		glNormal3d(0.0,1.0,0.0);
		glTexCoord2f(1,0); glVertex3d(x,y,z);
		glTexCoord2f(1,txCoordZ); glVertex3d(x,y,-z);
		glTexCoord2f(0,txCoordZ); glVertex3d(-x,y,-z);
		glTexCoord2f(0,0); glVertex3d(-x,y,z);
	glEnd();

	//direita
	glBegin(GL_POLYGON);
		glNormal3d(1.0,0.0,0.0);
		glTexCoord2d(0,0); glVertex3d(x,y,z);
		glTexCoord2d(txCoordY,0); glVertex3d(x,0,z);
		glTexCoord2d(txCoordY,txCoordZ); glVertex3d(x,0,-z);
		glTexCoord2d(0,txCoordZ); glVertex3d(x,y,-z);
	glEnd();

	//esquerda
	glBegin(GL_POLYGON);
		glNormal3d(-1.0,0.0,0.0);
		glTexCoord2d(txCoordY,txCoordZ); glVertex3d(-x,0,-z);
		glTexCoord2d(txCoordY,0); glVertex3d(-x,0,z);
		glTexCoord2d(0,0); glVertex3d(-x,y,z);
		glTexCoord2d(0,txCoordZ); glVertex3d(-x,y,-z);
	glEnd();

	//tras
	glBegin(GL_POLYGON);
		glNormal3d(0.0,0.0,-1.0);
		glTexCoord2d(txCoordY,1); glVertex3d(x,y,-z);
		glTexCoord2d(0,1); glVertex3d(x,0,-z);
		glTexCoord2d(0,0); glVertex3d(-x,0,-z);
		glTexCoord2d(txCoordY,0); glVertex3d(-x,y,-z);
	glEnd();

	//frente direita
	glBegin(GL_POLYGON);
		glNormal3d(0.0,0.0,1.0);
		glTexCoord2d(txCoordYaux,1); glVertex3d(x,y_aux,z);
		glTexCoord2d(txCoordYaux,1-txCoordXaux); glVertex3d(x_aux,y_aux,z);
		glTexCoord2d(0,1-txCoordXaux); glVertex3d(x_aux,0,z);
		glTexCoord2d(0,1); glVertex3d(x,0,z);
	glEnd();

	//frente esquerda
	glBegin(GL_POLYGON);
		glNormal3d(0.0,0.0,1.0);
		glTexCoord2d(txCoordYaux,0); glVertex3d(-x,y_aux,z);
		glTexCoord2d(0,0); glVertex3d(-x,0,z);
		glTexCoord2d(0,txCoordXaux); glVertex3d(-x_aux,0,z);
		glTexCoord2d(txCoordYaux,txCoordXaux); glVertex3d(-x_aux,y_aux,z);
	glEnd();

	//frente cima centro
	glBegin(GL_POLYGON);
		glNormal3d(0.0,0.0,1.0);
		glTexCoord2d(txCoordYaux,1-txCoordXaux); glVertex3d(x_aux,y_aux,z);
		glTexCoord2d(txCoordY,1-txCoordXaux); glVertex3d(x_aux,y,z);
		glTexCoord2d(txCoordY,txCoordXaux); glVertex3d(-x_aux,y,z);
		glTexCoord2d(txCoordYaux,txCoordXaux); glVertex3d(-x_aux,y_aux,z);
	glEnd();

	//frente cima direita
	glBegin(GL_POLYGON);
		glNormal3d(0.0,0.0,1.0);
		glTexCoord2d(txCoordYaux,1-txCoordXaux); glVertex3d(x_aux,y_aux,z);
		glTexCoord2d(txCoordYaux,1); glVertex3d(x,y_aux,z);
		glTexCoord2d(txCoordY,1); glVertex3d(x,y,z);
		glTexCoord2d(txCoordY,1-txCoordXaux); glVertex3d(x_aux,y,z);
	glEnd();

	//frente cima esquerda
	glBegin(GL_POLYGON);
		glNormal3d(0.0,0.0,1.0);
		glTexCoord2d(txCoordYaux,txCoordXaux); glVertex3d(-x_aux,y_aux,z);
		glTexCoord2d(txCoordY,txCoordXaux); glVertex3d(-x_aux,y,z);
		glTexCoord2d(txCoordY,0); glVertex3d(-x,y,z);
		glTexCoord2d(txCoordYaux,0); glVertex3d(-x,y_aux,z);
	glEnd();

	glDisable(GL_TEXTURE_2D);
}

void conveyorBelt(double x, double y, double z){
	float txCoordZ = 2*z/y;
	float txCoordX = 2*x/y;

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,TX_WOOD_BELT);
	//direita
	glBegin(GL_POLYGON);
		glNormal3d(1.0,0.0,0.0);
		glTexCoord2f(txCoordZ,1); glVertex3d(x,y,z);
		glTexCoord2f(txCoordZ,0); glVertex3d(x,0,z);
		glTexCoord2f(0,0); glVertex3d(x,0,-z);
		glTexCoord2f(0,1); glVertex3d(x,y,-z);
	glEnd();

	//esquerda
	glBegin(GL_POLYGON);
		glNormal3d(-1.0,0.0,0.0);
		glTexCoord2f(0,0); glVertex3d(-x,0,-z);
		glTexCoord2f(txCoordZ,0); glVertex3d(-x,0,z);
		glTexCoord2f(txCoordZ,1); glVertex3d(-x,y,z);
		glTexCoord2f(0,1); glVertex3d(-x,y,-z);
	glEnd();

	//frente
	glBegin(GL_POLYGON);
		glNormal3d(0.0,0.0,1.0);
		glTexCoord2f(txCoordX,1); glVertex3d(x,y,z);
		glTexCoord2f(0,1); glVertex3d(-x,y,z);
		glTexCoord2f(0,0); glVertex3d(-x,0,z);
		glTexCoord2f(txCoordX,0); glVertex3d(x,0,z);
	glEnd();

	glDisable(GL_TEXTURE_2D);
}

void belt(double x, double y, double z){
	//topo
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,TX_BELT);

	glBegin(GL_POLYGON);
		glNormal3d(0.0,1.0,0.0);
		glTexCoord2f(1,beltTxcoordZ-txInc); glVertex3d(x,y,z);
		glTexCoord2f(1,0-txInc); glVertex3d(x,y,-z);
		glTexCoord2f(0,0-txInc); glVertex3d(-x,y,-z);
		glTexCoord2f(0,beltTxcoordZ-txInc); glVertex3d(-x,y,z);
	glEnd();

	glDisable(GL_TEXTURE_2D);
}

void guillotineSupport(double side, double height){
	double temp = side/2.0;
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matMetal_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  matMetal_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   matMetal_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   matMetal_ambient);

	//frente
	glBegin(GL_POLYGON);
		glNormal3d(0,0,1);
		glVertex3d(-temp,0,temp);
		glVertex3d(temp,0,temp);
		glVertex3d(temp,height,temp);
		glVertex3d(-temp,height,temp);
	glEnd();

	//tras
	glBegin(GL_POLYGON);
		glNormal3d(0,0,-1);
		glVertex3d(-temp,0,-temp);
		glVertex3d(-temp,height,-temp);
		glVertex3d(temp,height,-temp);
		glVertex3d(temp,0,-temp);
	glEnd();

	//direita
	glBegin(GL_POLYGON);
		glNormal3d(1,0,0);
		glVertex3d(temp,0,temp);
		glVertex3d(temp,0,-temp);
		glVertex3d(temp,height,-temp);
		glVertex3d(temp,height,temp);
	glEnd();

	//esquerda
	glBegin(GL_POLYGON);
		glNormal3d(1,0,0);
		glVertex3d(-temp,0,temp);
		glVertex3d(-temp,height,temp);
		glVertex3d(-temp,height,-temp);
		glVertex3d(-temp,0,-temp);
	glEnd();

	//cima
	glBegin(GL_POLYGON);
		glNormal3d(0,1,0);
		glVertex3d(temp,height,temp);
		glVertex3d(temp,height,-temp);
		glVertex3d(-temp,height,-temp);
		glVertex3d(-temp,height,temp);
	glEnd();
}

void guillotine(double x, double y){
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matMetal_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  matMetal_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   matMetal_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   matMetal_ambient);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,TX_GUILLOTINE);

	glBegin(GL_POLYGON);
		glNormal3d(0,0,1);
		glTexCoord2f(guillotineTxX,1); glVertex3d(x,y,0);
		glTexCoord2f(0,1); glVertex3d(-x,y,0);
		glTexCoord2f(0,0); glVertex3d(-x,-y,0);
		glTexCoord2f(guillotineTxX,0); glVertex3d(x,-y,0);
	glEnd();

	glBegin(GL_POLYGON);
		glNormal3d(0,0,-1);
		glTexCoord2f(guillotineTxX,1); glVertex3d(x,y,0);
		glTexCoord2f(guillotineTxX,0); glVertex3d(x,-y,0);
		glTexCoord2f(0,0); glVertex3d(-x,-y,0);
		glTexCoord2f(0,1); glVertex3d(-x,y,0);
	glEnd();

	glDisable(GL_TEXTURE_2D);
}

//coordenadas do centro da maquina
void printingMachine(double x, double z){
	glPushMatrix();

	glTranslated(x,0,z);
	machine(machineX,machineY,machineZ,machineXaux,machineYaux);
	
	glPushMatrix();
	glTranslated(-guillotineSupportX,guillotineSupportY,guillotineSupportZ);
	guillotineSupport(guillotineSupportSide,guillotineSupportHeight);
	glPopMatrix();

	glPushMatrix();
	glTranslated(guillotineSupportX,guillotineSupportY,guillotineSupportZ);
	guillotineSupport(guillotineSupportSide,guillotineSupportHeight);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0.0,machineY + teapotSize/2.0,0.0);
	glFrontFace(GL_CW);
	glutSolidTeapot(teapotSize);
	glFrontFace(GL_CCW);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0,0,conveyorZ+machineZ);
	conveyorBelt(conveyorX,conveyorY,conveyorZ);
	glPopMatrix();

	glPopMatrix();
}

void platen(){
	glPushMatrix();
	glRotated(90.0,1,0,0);
	glRotated(-90.0,0,1,0);
	myCylinderTexture(platenRadius,platenHeight,platenSlices,TRUE,TX_NEWSPAPER);
	glPopMatrix();
}
////////////////////////////////////////////////////////////
//ROBOT

void robotBody(double x, double y, double z){
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matMetal_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  matMetal_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   matMetal_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   matMetal_ambient);

	//frente
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, TX_METAL);

	float txCoordX = 2*x/y;
	float txCoordZ = 2*z/y;

	glBegin(GL_POLYGON);
		glNormal3d(0,0,1);
		glTexCoord2f(0,0); glVertex3d(x,0,z);
		glTexCoord2f(0,1); glVertex3d(x,y,z);
		glTexCoord2f(txCoordX,1); glVertex3d(-x,y,z);
		glTexCoord2f(txCoordX,0); glVertex3d(-x,0,z);
	glEnd();

	glBegin(GL_POLYGON);
		glNormal3d(0,0,-1);
		glTexCoord2f(0,0); glVertex3d(x,0,z);
		glTexCoord2f(txCoordX,0); glVertex3d(-x,0,z);
		glTexCoord2f(txCoordX,1); glVertex3d(-x,y,z);
		glTexCoord2f(0,1); glVertex3d(x,y,z);
	glEnd();

	//direita
	glBegin(GL_POLYGON);
		glNormal3d(1,0,0);
		glTexCoord2f(0,0); glVertex3d(x,0,z);
		glTexCoord2f(txCoordZ,0); glVertex3d(x,0,-z);
		glTexCoord2f(txCoordZ,1); glVertex3d(x,y,-z);
		glTexCoord2f(0,1); glVertex3d(x,y,z);
	glEnd();

	glBegin(GL_POLYGON);
		glNormal3d(-1,0,0);
		glTexCoord2f(0,0); glVertex3d(x,0,z);
		glTexCoord2f(0,1); glVertex3d(x,y,z);
		glTexCoord2f(txCoordX,1); glVertex3d(x,y,-z);
		glTexCoord2f(txCoordX,0); glVertex3d(x,0,-z);
	glEnd();

	//esquerda
	glBegin(GL_POLYGON);
		glNormal3d(-1,0,0);
		glTexCoord2f(0,0); glVertex3d(-x,0,z);
		glTexCoord2f(0,1); glVertex3d(-x,y,z);
		glTexCoord2f(txCoordZ,1); glVertex3d(-x,y,-z);
		glTexCoord2f(txCoordZ,0); glVertex3d(-x,0,-z);
	glEnd();

	glBegin(GL_POLYGON);
		glNormal3d(1,0,0);
		glTexCoord2f(0,0); glVertex3d(-x,0,z);
		glTexCoord2f(txCoordZ,0); glVertex3d(-x,0,-z);
		glTexCoord2f(txCoordZ,1); glVertex3d(-x,y,-z);
		glTexCoord2f(0,1); glVertex3d(-x,y,z);
	glEnd();

	//tras
	glBegin(GL_POLYGON);
		glNormal3d(0,0,-1);
		glTexCoord2f(0,0); glVertex3d(x,0,-z);
		glTexCoord2f(txCoordX,0); glVertex3d(-x,0,-z);
		glTexCoord2f(txCoordX,1); glVertex3d(-x,y,-z);
		glTexCoord2f(0,1); glVertex3d(x,y,-z);
	glEnd();

	glBegin(GL_POLYGON);
		glNormal3d(0,0,1);
		glTexCoord2f(0,0); glVertex3d(x,0,-z);
		glTexCoord2f(0,1); glVertex3d(x,y,-z);
		glTexCoord2f(txCoordX,1); glVertex3d(-x,y,-z);
		glTexCoord2f(txCoordX,0); glVertex3d(-x,0,-z);
	glEnd();

	//baixo
	glBegin(GL_POLYGON);
		glNormal3d(0,1,0);
		glTexCoord2f(0,0); glVertex3d(x,0,z);
		glTexCoord2f(0,txCoordZ); glVertex3d(x,0,-z);
		glTexCoord2f(txCoordX,txCoordZ); glVertex3d(-x,0,-z);
		glTexCoord2f(txCoordX,0); glVertex3d(-x,0,z);
	glEnd();

	glDisable(GL_TEXTURE);
}

void robotLight(double height, double radius,double outerRadius){
	
	glPushMatrix();
	glEnable(GL_CLIP_PLANE0);
	glTranslated(outerRadius,height,0);
	glClipPlane(GL_CLIP_PLANE0,robotClipPlane);
	glutSolidTorus(radius,outerRadius,20,20);
	glDisable(GL_CLIP_PLANE0);
	glPopMatrix();

	glPushMatrix();
	glRotated(-90,1,0,0);
	gluCylinder(glQ,radius,radius,height,20,20);
	glPopMatrix();
}

void robot(double y){
		
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matWhite_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  matWhite_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   matWhite_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   matWhite_ambient);

	glPushMatrix();
	glTranslated(robotWheelX,robotWheelRadius,robotBodyZ+.01);
	myCylinder(robotWheelRadius,robotWheelHeight,robotWHeelSlices,TRUE);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-robotWheelX,robotWheelRadius,robotBodyZ+0.01);
	myCylinder(robotWheelRadius,robotWheelHeight,robotWHeelSlices,TRUE);
	glPopMatrix();

	glPushMatrix();
	glTranslated(robotWheelX,robotWheelRadius,-robotBodyZ -robotWheelHeight-.01);
	myCylinder(robotWheelRadius,robotWheelHeight,robotWHeelSlices,TRUE);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-robotWheelX,robotWheelRadius,-robotBodyZ -robotWheelHeight-.01);
	myCylinder(robotWheelRadius,robotWheelHeight,robotWHeelSlices,TRUE);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0,y,0);
	
	glPushMatrix();
	glTranslated(robotLightX,0,robotLightZ);
	robotLight(robotLightHeight,robotLightRadius,robotLightOuterRadius);
	glPopMatrix();
	
	robotBody(robotBodyX,robotBodyY,robotBodyZ);
	glPopMatrix();
}
////////////////////////////////////////////////////////////
//JORNAIS
void drawNewspapers(double x, double y,double zi, double zf){
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matWhite_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  matWhite_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   matWhite_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   matWhite_ambient);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,TX_NEWSPAPER);

	double txZ= (zf-zi)/(2*x);

	glBegin(GL_POLYGON);
		glNormal3d(0,1,0);
		glTexCoord2f(0,0+newspapersTxInc); glVertex3d(-x, y, zf);
		glTexCoord2f(1,0+newspapersTxInc); glVertex3d(x, y, zf);
		glTexCoord2f(1,txZ+newspapersTxInc); glVertex3d(x, y, zi);
		glTexCoord2f(0,txZ+newspapersTxInc); glVertex3d(-x, y, zi);
	glEnd();
	glDisable(GL_TEXTURE_2D);
}

void drawNewspaper(double x, double y,double zi, double zf){
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matWhite_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  matWhite_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   matWhite_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   matWhite_ambient);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,TX_NEWSPAPER);

	glBegin(GL_POLYGON);
		glNormal3d(0,1,0);
		glTexCoord2f(0,0); glVertex3d(-x, y, zf);
		glTexCoord2f(1,0); glVertex3d(x, y, zf);
		glTexCoord2f(1,newspaperTxInc); glVertex3d(x, y, zi);
		glTexCoord2f(0,newspaperTxInc); glVertex3d(-x, y, zi);
	glEnd();
	glDisable(GL_TEXTURE_2D);
}

void paper(){
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matWhite_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  matWhite_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   matWhite_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   matWhite_ambient);
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D,TX_NEWSPAPER);

	glBegin(GL_POLYGON);
		glNormal3d(0,1,0);
		glTexCoord2f(0,0); glVertex3d(-machineXaux, 0, machineXaux);
		glTexCoord2f(1,0); glVertex3d(machineXaux, 0,machineXaux);
		glTexCoord2f(1,1); glVertex3d(machineXaux, 0, -machineXaux);
		glTexCoord2f(0,1); glVertex3d(-machineXaux, 0, -machineXaux);
	glEnd();
	glDisable(GL_TEXTURE_2D);
}
/////////////////////////////////////////////////////////////

void myUpdateTransforms(int dummy)
{
	switch(st){
		case ST00:
			txInc+=txinc;
			if(txInc>conveyorZ/conveyorX)
				txInc=0;

			platen_ang+=platen_delta_ang;
			if(platen_ang>=360)
				platen_ang=0;

			newspapersZf+=belt_delta_z;

			if(!(newspapersZf<0)){//metade do tapete
				if(guillotineUp){
					guillotineYa+=guillotineInc;
					if(guillotineYa>=guillotinePosYf)
						guillotineUp=FALSE;
				}
				else{
					guillotineYa-=guillotineInc;
					if(guillotineYa<=guillotinePosYi)
						guillotineUp=TRUE;
				}
			}

			if(!(newspapersZf<conveyorZ))
				st=ST01;
			break;
		case ST01:
			if(guillotineUp){
				guillotineYa+=guillotineInc;
				if(guillotineYa>=guillotinePosYf)
					guillotineUp=FALSE;
			}
			else{
				guillotineYa-=guillotineInc;
				if(guillotineYa<=guillotinePosYi)
					guillotineUp=TRUE;
			}

			txInc+=txinc;
			if(txInc>conveyorZ/conveyorX)
				txInc=0;

			platen_ang+=platen_delta_ang;
			if(platen_ang>=360)
				platen_ang=0;

			newspaperZf+=belt_delta_z;
			newspaperTxInc+=txinc;
			newspapersTxInc+=txinc;
			if(!(newspaperZf<newspaperZF)){
				st=ST02;
				newspaperZf=newspaperZi;
				newspapersTxInc=newspaperTxInc=0;
				platen_ang=0;
				guillotineYa=guillotinePosYi;
				guillotineUp=TRUE;
			}
			break;
		case ST02:
			if(paperZa<paperZf)
				paperZa+=belt_delta_z;
			else if(paperYa>paperYf)
				paperYa-=belt_delta_z;
			else{
				++n_newspapers;
				paperZa = paperZi;
				paperYa = paperYi;
				if(n_newspapers==newspapersTotal){
					st=ST1;
				}
				else
					st=ST01;
			}
			break;
		case ST1:
			if(robotXa<centerX)
				cam3RefX= robotXa +=delta_x;
			else{
				st=ST2;
			}
			break;
		case ST2:
			if(ang<=90){
				ang+=delta_ang;
				ang_radians=ang*pi/180.0;
				cam3RefX = centerX+trajectoryRadius*sin(ang_radians);
				cam3RefZ = centerZ+trajectoryRadius*cos(ang_radians);
			}
			else{
				st=ST3;
				ang=0.0;
				robotXa=robotXf;
				robotZa=centerZ;
			}
			break;
		case ST3:
			if(robotZa>robotZf)
				cam3RefZ = robotZa-=delta_z;
			else{
				n_newspapers=0;
				st=ST4;
			}
			break;
		case ST4:
			if(robotZa<centerZ)
				cam3RefZ = robotZa+=delta_z;
			else{
				st=ST5;
			}
			break;
		case ST5:
			if(ang<=90){
				ang+=delta_ang;
				ang_radians=half_pi-ang*pi/180.0;
				cam3RefX = centerX+trajectoryRadius*sin(ang_radians);
				cam3RefZ = centerZ+trajectoryRadius*cos(ang_radians);
			}
			else{
				st=ST6;
				ang=0.0;
				robotXa=centerX;
				robotZa=robotZi;
			}
			break;
		case ST6:
			if(robotXa>robotXi)
				cam3RefX = robotXa-=delta_x;
			else{
				st=ST01;
			}
			break;
	}

	glutTimerFunc(delta_t, myUpdateTransforms, 0);
}

void setCamera(){
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	glFrustum( -xy_aspect*.04, xy_aspect*.04, -.04, .04, .1, 500.0 );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	
	switch(cam){
	case 1:
		glTranslatef( obj_pos[0], obj_pos[1], -obj_pos[2]-70 );
		
		// roda a cena para ficar em perspectiva
		glRotated( 20.0, 1.0,0.0,0.0 );		// 20 graus em torno de X
		glRotated(-10.0, 0.0,1.0,0.0 );		//-10 graus em torno de Y

		// roda a cena de acordo com o botao (esfera) de rotacao
		glMultMatrixf( view_rotate );
		break;
	case 2:
		gluLookAt(cam2PosX,cam2PosY,cam2PosZ,cam2RefX,cam2RefY,cam2RefZ,cam2VectX,cam2VectY,cam2VectZ);
		break;
	case 3:
		gluLookAt(cam3PosX,cam3PosY,cam3PosZ,cam3RefX,cam3RefY,cam3RefZ,cam3VectX,cam3VectY,cam3VectZ);
		break;
	}
}

void showCameraNumber(){
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0,width,0,height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	glColor3f(1.0,1.0,0.0);
	glRasterPos3f(10,height-30,0);
	
	glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, '0'+cam);
	
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glEnable(GL_LIGHTING);
}

void display(void)
{
	// ****** declaracoes internas 'a funcao display() ******
	
	float temp;

	// ****** fim de todas as declaracoes da funcao display() ******

	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
	setCamera();
	
	// permissao de atribuicao directa de cores
	// para objectos que nao tem material atribuido, como
	// e' o caso dos eixos e da esfera que simboliza a fonte de luz...
	/*glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);*/

	// Actualizacao da posicao da fonte de luz...
	light0_position[0] = light0x;	// por razoes de eficiencia, os restantes 
	light0_position[1] = light0y;	// parametros _invariaveis_ da LIGHT0 mantem os valores
	light0_position[2] = light0z;	// definidos na funcao de inicializacao
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

	// ... e da esfera que a simboliza
	/*glColor3f(1.0,1.0,0.0);		// cor amarela
	gluQuadricOrientation( glQ, GLU_INSIDE);
	glPushMatrix();
	glTranslated(light0x,light0y,light0z);
	gluSphere(glQ, symb_light0_radius, symb_light0_slices, symb_light0_stacks);
    glPopMatrix();
	gluQuadricOrientation( glQ, GLU_OUTSIDE);

	glDisable(GL_COLOR_MATERIAL);*/
	
	glCallList(DP_GEOMETRY);
	glCallList(DP_MACHINE);
	
	glPushMatrix();

	if(st==ST2){
		glTranslated(centerX,0,centerZ);
		glRotated(ang,0,1,0);
		glTranslated(-centerX,0,-centerZ);
	}
	else if(st==ST5){
		glTranslated(centerX,0,centerZ);
		glRotated(-ang,0,1,0);
		glTranslated(-centerX,0,-centerZ);
	}

	glTranslated(robotXa,0,robotZa);
	if(st==ST3||st==ST4||st==ST5)
		glRotated(90,0,1,0);

	light1_position[0] = light1x;	 
	light1_position[1] = light1y;	
	light1_position[2] = light1z;	

	glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, light1_cone_radius);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, light1_spot_direction);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);

	if(n_newspapers>0){
		glPushMatrix();
		glTranslated(0,paperYf,0);
		paper();
		glPopMatrix();
	}
	glCallList(DP_ROBOT);
	glPopMatrix();

	glPushMatrix();
	glTranslated(printingMachineX,0,printingMachineZ+conveyorZ+machineZ);
	belt(conveyorX,conveyorY,conveyorZ);
	drawNewspapers(newspapersX,newspapersY,newspapersZi,newspapersZf);
	glPopMatrix();

	if(st==ST01){
		glPushMatrix();
		glTranslated(newspaperX,0,newspaperZ);
		drawNewspaper(newspapersX,newspapersY,newspaperZi,newspaperZf);
		glPopMatrix();
	}

	if(st==ST02){
		glPushMatrix();
		glTranslated(newspaperX,paperYa,paperZa);
		paper();
		glPopMatrix();
	}

	glPushMatrix();
	glTranslated(platenX,platenY,platenZ);
	glRotated(-platen_ang,1.0,0.0,0.0);
	platen();
	glPopMatrix();

	glPushMatrix();
	glTranslated(guillotinePosX,guillotineYa,guillotinePosZ);	
	guillotine(guillotineX,guillotineY);
	glPopMatrix();

	showCameraNumber();

	glEnable(GL_NORMALIZE);
	// swapping the buffers causes the rendering above to be shown
	glutSwapBuffers();
	// glFlush(); included in the above glutSwapBuffers()
}

/* Mouse handling */
void processMouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{	 
	}
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
	{	
	}
	
	glutPostRedisplay();
	
}

void processMouseMoved(int x, int y)
{
	
	// pedido de refrescamento da janela
	glutPostRedisplay();				

}

void processPassiveMouseMoved(int x, int y)
{

	// pedido de refrescamento da janela
	glutPostRedisplay();				
}

void reshape(int w, int h)
{
	width=w;
	height=h;

	int tx, ty, tw, th;

	GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
	glViewport( tx, ty, tw, th );
	xy_aspect = (float)tw / (float)th;

	glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y)
{
   switch (key) {
      case 27:		// tecla de escape termina o programa
         exit(0);
         break;
	  case '1':
		  cam=1;
		  view_rot->enable();
		  trans_z->enable();
		  break;
	  case '2':
		  cam=2;
		  view_rot->disable();
		  trans_z->disable();
		  break;
	  case '3':
		  cam=3;
		  view_rot->disable();
		  trans_z->disable();
		  break;
	  case 'j':
		  if(cam==3)
			  --cam3PosX;
		  break;
	  case 'l':
		  if(cam==3)
			  ++cam3PosX;
		  break;
	  case 'k':
		  if(cam==3)
			  --cam3PosY;
		  break;
	  case 'i':
		  if(cam==3)
			  ++cam3PosY;
		  break;
	  case 'q':
		  if(cam==3)
			  --cam3PosZ;
		  break;
	  case 'a':
		  if(cam==3)
			  ++cam3PosZ;
		  break;
   }
}

void myGlutIdle( void )
{
  /* According to the GLUT specification, the current window is 
     undefined during an idle callback.  So we need to explicitly change
     it if necessary */
  if ( glutGetWindow() != main_window ) 
    glutSetWindow(main_window);  


  glutPostRedisplay();

}

void inicializacao()
{

	// inicialização de apontador para quádricas
	glQ = gluNewQuadric();


	glFrontFace(GL_CCW);		// Front faces defined using a counterclockwise rotation
	glDepthFunc(GL_LEQUAL);		// Por defeito e GL_LESS
	glEnable(GL_DEPTH_TEST);	// Use a depth (z) buffer to draw only visible objects


	// Face Culling para aumentar a velocidade
	glEnable(GL_CULL_FACE);
	//glCullFace(GL_BACK);		// GL_FRONT, GL_BACK, GL_FRONT_AND_BACK


	// Define que modelo de iluminacao utilizar; consultar o manual de referencia
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_ambient);  // define luz ambiente
	glLightModelf (GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
	glLightModeli (GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
	
	// por defeito a cor de fundo e o preto
	// glClearColor(1.0,1.0,1.0,1.0);    // cor de fundo a branco


	// declaracoes para a fonte luz GL_LIGHT0
	glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
	glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION,  light0_kc);
	glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION,    light0_kl);
	glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, light0_kq);

	// declaracoes para a fonte luz GL_LIGHT1
	glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
	glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION,  light1_kc);
	glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION,    light1_kl);
	glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, light1_kq);

	// Permitir calculos de iluminacao
	glEnable(GL_LIGHTING);
	// "Acender" a fonte de luz GL_LIGHT0
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);


	// Declaracoe para shading
	glShadeModel(GL_SMOOTH);			// GL_FLAT / GL_SMOOTH
	glPolygonMode(GL_FRONT, GL_FILL);	// preence a face da frente dos poligonos
	//glPolygonMode(GL_FRONT, GL_LINE);	// desenha arestas dos poligonos

	pixmap.readBMPFile("newspaper.bmp");
	pixmap.setTexture(TX_NEWSPAPER);

	pixmap.readBMPFile("paisagem.bmp");
	pixmap.setTexture(TX_LANDSCAPE);

	pixmap.readBMPFile("metal.bmp");
	pixmap.setTexture(TX_METAL);

	pixmap.readBMPFile("wood_machine.bmp");
	pixmap.setTexture(TX_WOOD_MACHINE);

	pixmap.readBMPFile("wood.bmp");
	pixmap.setTexture(TX_WOOD_BELT);

	pixmap.readBMPFile("belt.bmp");
	pixmap.setTexture(TX_BELT);

	pixmap.readBMPFile("floor.bmp");
	pixmap.setTexture(TX_FLOOR);

	pixmap.readBMPFile("guillotine.bmp");
	pixmap.setTexture(TX_GUILLOTINE);

	pixmap.readBMPFile("wall.bmp");
	pixmap.setTexture(TX_WALL);

	glNewList(DP_GEOMETRY,GL_COMPILE);
		impostors();
		myRoom(roomX,roomY,roomZ,windowYi,windowYf,windowZi,windowZf,doorXi,doorXf,doorY,curvedXHeight,curvedNSlices,curvedYSlices);
	glEndList();

	glNewList(DP_MACHINE,GL_COMPILE);
		printingMachine(printingMachineX,printingMachineZ);
	glEndList();

	glNewList(DP_ROBOT,GL_COMPILE);
		robot(robotY);
	glEndList();
}

void terminacao()
{
	gluDeleteQuadric(glQ);
}

void turn_light(){
	if(light1_active){
		light1_active=FALSE;
		glDisable(GL_LIGHT1);
	}
	else{
		light1_active=TRUE;
		glEnable(GL_LIGHT1);
	}
}

int main(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize (DIMX, DIMY);
	glutInitWindowPosition (INITIALPOS_X, INITIALPOS_Y);
	main_window = glutCreateWindow (argv[0]);
 
	glutDisplayFunc(display);
	GLUI_Master.set_glutReshapeFunc(reshape);
	GLUI_Master.set_glutKeyboardFunc (keyboard);
	GLUI_Master.set_glutMouseFunc(processMouse);
	glutMotionFunc(processMouseMoved);
	glutPassiveMotionFunc(processPassiveMouseMoved);
	GLUI_Master.set_glutSpecialFunc( NULL );
   
	glutTimerFunc(delta_t, myUpdateTransforms, 0);

	/*** Create the bottom subwindow ***/
	glui2 = GLUI_Master.create_glui_subwindow( main_window, GLUI_SUBWINDOW_BOTTOM );
	glui2->set_main_gfx_window( main_window );

	view_rot = glui2->add_rotation( "Rotacao", view_rotate );
	view_rot->set_spin( 1.0 );
	
	glui2->add_column( false );
	trans_z = glui2->add_translation( "Zoom", GLUI_TRANSLATION_Z, &obj_pos[2] );
	trans_z->set_speed( .10 );

	glui2->add_column( false );
	GLUI_Button *light_switch = glui2->add_button("Luz",0,(GLUI_Update_CB) turn_light);

	/* We register the idle callback with GLUI, not with GLUT */
	GLUI_Master.set_glutIdleFunc( myGlutIdle );
   
	inicializacao();
   
	glutMainLoop();

	terminacao();

	return 0;
}