#include "stdAfx.h"
#include "OpenGLControl.h"

COpenGL::COpenGL(void)
{
}

COpenGL::~COpenGL(void)
{
	wglMakeCurrent(hDC, NULL);
	wglDeleteContext(hRC);
}

void COpenGL::Init(int width, int height) {
	// openGL�ĳ�ʼ������
	glClearColor(0.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_SMOOTH);
	//glViewport(0, 0, 200, 200);
	glMatrixMode(GL_PROJECTION);
	gluPerspective(60, (GLfloat)width / (GLfloat)height, 0.1, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

bool COpenGL::SetupPixelFormat(HDC hDC0) {
	int nPixelFormat;                 // ���ص��ʽ
	hDC = hDC0;
	PIXELFORMATDESCRIPTOR pfd = {
		sizeof(PIXELFORMATDESCRIPTOR),    // pfd�ṹ�Ĵ�С
		1,                                // �汾��
		PFD_DRAW_TO_WINDOW |              // ֧���ڴ����л�ͼ
		PFD_SUPPORT_OPENGL |              // ֧��OpenGL
		PFD_DOUBLEBUFFER,                 // ˫����ģʽ
		PFD_TYPE_RGBA,                    // RGBA ��ɫģʽ
		24,                               // 24 λ��ɫ���
		0, 0, 0, 0, 0, 0,                 // ������ɫλ
		0,                                // û�з�͸���Ȼ���
		0,                                // ������λλ
		0,                                // ���ۼӻ���
		0, 0, 0, 0,                       // �����ۼ�λ
		32,                               // 32 λ��Ȼ���   
		0,                                // ��ģ�建��
		0,                                // �޸�������
		PFD_MAIN_PLANE,                   // ����
		0,                                // ����
		0, 0, 0                           // ���Բ�,�ɼ��Ժ������ģ
	};
	if (!(nPixelFormat = ChoosePixelFormat(hDC, &pfd)))
	{
		MessageBox(NULL, L"can not find proper mode", L"Error", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;
	}
	SetPixelFormat(hDC, nPixelFormat, &pfd);
	hRC = wglCreateContext(hDC);
	wglMakeCurrent(hDC, hRC);
	return TRUE;
}
void COpenGL::Reshap(int width, int height) {
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective
	(60.0f,
		(GLfloat)width / (GLfloat)height,
		0.1f,
		100.0f
	);
	//gluLookAt(10,5,10,0,0,0,0,1,0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}
void COpenGL::Render() {
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0, 0.0, 0.0);
	glLoadIdentity();
	glTranslatef(0.0, 0.0, -5.0);
	glBegin(GL_TRIANGLES);
	glVertex3f(0.0, 1.0, 0.0);
	glVertex3f(-1.0, -1.0, 0.0);
	glVertex3f(1.0, -1.0, 0.0);
	glEnd();
	SwapBuffers(hDC);
}
