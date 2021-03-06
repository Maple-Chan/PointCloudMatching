
// PointCloudMatchinDlg.cpp: 实现文件
//

#include "stdafx.h"
#include "PointCloudMatchin.h"
#include "PointCloudMatchinDlg.h"
#include "afxdialogex.h"
#include "ReadFile.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CPointCloudMatchinDlg 对话框



CPointCloudMatchinDlg::CPointCloudMatchinDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_POINTCLOUDMATCHIN_DIALOG, pParent)
{

	manager = new RoamingScenceManager();

	midButtonState = false;
	ctrlKeyState = false;
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CPointCloudMatchinDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CPointCloudMatchinDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_WM_MOUSEMOVE()
	//ON_WM_LBUTTONDOWN()
	//ON_WM_RBUTTONDOWN()
	ON_WM_MOUSEWHEEL()
	ON_WM_MBUTTONDOWN()
	ON_WM_MBUTTONUP()
	ON_BN_CLICKED(IDC_BUTTON1, &CPointCloudMatchinDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CPointCloudMatchinDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CPointCloudMatchinDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &CPointCloudMatchinDlg::OnBnClickedButton4)
END_MESSAGE_MAP()



// CPointCloudMatchinDlg 消息处理程序

BOOL CPointCloudMatchinDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	/*CRect temprect(0, 0, 640, 480);
	CWnd::SetWindowPos(NULL, 0, 0, temprect.Width(), temprect.Height(), SWP_NOZORDER | SWP_NOMOVE);*/
	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	///////////////////////OPENGL INIT///////////////////////// 
	CWnd *wnd = GetDlgItem(IDC_OPENGL);
	hrenderDC = ::GetDC(wnd->m_hWnd);
	if (SetWindowPixelFormat(hrenderDC) == FALSE)
		return 0;

	if (CreateViewGLContext(hrenderDC) == FALSE)
		return 0;

	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_FILL);
	/////////////////////////////////////////// 
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);
	
	CDialog *pDlg = (CDialog *)GetDlgItem(IDC_OPENGL);
	/*CRect rt;
	pDlg->GetWindowRect(&rt);//big包含系统菜单*/
	CRect crt;
	pDlg->GetClientRect(&crt);//small 不包含系统菜单


	glViewport(0,0,crt.Width() ,crt.Height());
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-200,200,-200,200,-1000,1000);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glShadeModel(GL_SMOOTH);       // Enable Smooth Shading 
	glClearColor(1.0f, 1.0f, 1.0f, 0.5f);    // Black Background 
	glClearDepth(1.0f);         // Depth Buffer Setup 
	glEnable(GL_DEPTH_TEST);       // Enables Depth Testing 
	glDepthFunc(GL_LEQUAL); // The Type Of Depth Testing To Do 

	manager->init();

	//glutMouseFunc(Mouse);
	//glutMotionFunc(onMouseMove);
							/////////////////////////////////////////////////////////////////////////
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	SetTimer(1, 10, 0);

	////////////////////////////////////////////////////////////////
	///-----------------------------------------------------



	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CPointCloudMatchinDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CPointCloudMatchinDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}
GLint Rotateadd = 0;

void CPointCloudMatchinDlg::OnTimer(UINT nIDEvent) //实时绘制场景 
{
	// TODO: Add your message handler code here and/or call default 
	RenderScene();
	m_yRotate += Rotateadd;
	CDialog::OnTimer(nIDEvent);
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CPointCloudMatchinDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

readFile newFile;
mPointCloud newPointCloud;

void CPointCloudMatchinDlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	// 读取数据
	newFile.readCloud();
	newPointCloud.Center=newPointCloud.computeGravityCenter();
	mPointCloud Points(newFile.cloud1, newFile.cloud2);//定义点云
	newPointCloud = Points;//将值传给全局变量
	
	

	//下面进行显示初始化的点云
}


void CPointCloudMatchinDlg::OnBnClickedButton2()
{
	// TODO: 在此添加控件通知处理程序代码
	// ICP配准
	newPointCloud.ICP4KDTree(0.8,30);//进行ICP迭代
	//newFile.writeCloud(newPointCloud.getCloud2());//将ICP迭代完的cloud2输出到文件中

	//下面进行显示ICP变换后的点云
}
BOOL CPointCloudMatchinDlg::SetWindowPixelFormat(HDC hDC)
{
	PIXELFORMATDESCRIPTOR pixelDesc;

	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR);
	pixelDesc.nVersion = 1;

	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW |
		PFD_SUPPORT_OPENGL |
		PFD_DOUBLEBUFFER |
		PFD_TYPE_RGBA;

	pixelDesc.iPixelType = PFD_TYPE_RGBA;
	pixelDesc.cColorBits = 32;
	pixelDesc.cRedBits = 0;
	pixelDesc.cRedShift = 0;
	pixelDesc.cGreenBits = 0;
	pixelDesc.cGreenShift = 0;
	pixelDesc.cBlueBits = 0;
	pixelDesc.cBlueShift = 0;
	pixelDesc.cAlphaBits = 0;
	pixelDesc.cAlphaShift = 0;
	pixelDesc.cAccumBits = 0;
	pixelDesc.cAccumRedBits = 0;
	pixelDesc.cAccumGreenBits = 0;
	pixelDesc.cAccumBlueBits = 0;
	pixelDesc.cAccumAlphaBits = 0;
	pixelDesc.cDepthBits = 0;
	pixelDesc.cStencilBits = 1;
	pixelDesc.cAuxBuffers = 0;
	pixelDesc.iLayerType = PFD_MAIN_PLANE;
	pixelDesc.bReserved = 0;
	pixelDesc.dwLayerMask = 0;
	pixelDesc.dwVisibleMask = 0;
	pixelDesc.dwDamageMask = 0;

	PixelFormat = ChoosePixelFormat(hDC, &pixelDesc);
	if (PixelFormat == 0) // Choose default 
	{
		PixelFormat = 1;
		if (DescribePixelFormat(hDC, PixelFormat,
			sizeof(PIXELFORMATDESCRIPTOR), &pixelDesc) == 0)
		{
			return FALSE;
		}
	}

	if (SetPixelFormat(hDC, PixelFormat, &pixelDesc) == FALSE)

	{
		return FALSE;
	}

	return TRUE;
}
BOOL CPointCloudMatchinDlg::CreateViewGLContext(HDC hDC)
{
	hrenderRC = wglCreateContext(hDC);

	if (hrenderRC == NULL)
		return FALSE;

	if (wglMakeCurrent(hDC, hrenderRC) == FALSE)
		return FALSE;

	return TRUE;
}

void CPointCloudMatchinDlg::RenderScene()
{

	///////////////////////////////////////////////// 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	//glLoadIdentity();
	
	//gluLookAt(r*cos(c*du), h, r*sin(c*du), 0, 0, 0, 0, 1, 0); //从视点看远点,y轴方向(0,1,0)是上方向

	manager->render();
	
	glTranslatef(0.0f, 0.0f, -6.0f);      // Move Left 1.5 Units And Into The Screen 6.0 
	//m_yRotate = 50;
	glRotated(m_yRotate, newPointCloud.Center.x, newPointCloud.Center.y, newPointCloud.Center.z);//后三个参数是设置重心

	glBegin(GL_POINTS); // Drawing Using Triangles 
	vector<PointM> cloud5;
	vector<PointM> cloud6;
	cloud5 = newPointCloud.getCloud1();
	int size5 = newPointCloud.getCloud1().size();

	cloud6 = newPointCloud.getCloud2();
	int size6 = newPointCloud.getCloud2().size();


	int i = 0;
	glColor3f(1.0, 1.0,1.0);
	for (i = 0; i < size5; i++)
	{
		glVertex3f(cloud5[i].x, cloud5[i].y, cloud5[i].z);
	}
	int j = 0;
	glColor3f(1.0, 0.0, 0.0);
	for (j = 0; j < size6; j++)
	{
		glVertex3f(cloud6[j].x, cloud6[j].y, cloud6[j].z);
	}
	/*glVertex3f(0.0f, 1.0f, 0.0f);     // Top 
	glVertex3f(-1.0f, -1.0f, 0.0f);     // Bottom Left 
	glVertex3f(1.0f, -1.0f, 0.0f);     // Bottom Right */
	glEnd();           // Finished Drawing The Triangle 
	SwapBuffers(hrenderDC);


}

void CPointCloudMatchinDlg::OnSize(UINT nType, int cx, int cy)
{

}

void CPointCloudMatchinDlg::OnBnClickedButton3()
{
	// TODO: 在此添加控件通知处理程序代码
	Rotateadd = 0;
}


void CPointCloudMatchinDlg::OnBnClickedButton4()
{
	// TODO: 在此添加控件通知处理程序代码
	Rotateadd = 1;
}



/*void CPointCloudMatchinDlg::OnMButtonDown(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值  
	midButtonState = true;
}*/
//void CPointCloudMatchinDlg::OnLButtonDown(UINT nFlags, CPoint point)
//{
//	midButtonState = true;
//}
//void CPointCloudMatchinDlg::OnRButtonDown(UINT nFlags, CPoint point)
//{
//	midButtonState = false;
//}
/*void CPointCloudMatchinDlg::OnMButtonUp(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值  

	midButtonState = false;
}*/


/*void CPointCloudMatchinDlg::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值  
	//改变这上面的参数
	//写鼠标拖动进行旋转的逻辑
	long x =point.x;
	long y = point.y;
	if (midButtonState)
	{
		du += x - oldmx; //鼠标在窗口x轴方向上的增量加到视点绕y轴的角度上，这样就左右转了
		h += 2.0*(y - oldmy); //鼠标在窗口y轴方向上的改变加到视点的y坐标上，就上下转了
		if (h>1.0f) h = 1.0f; //视点y坐标作一些限制，不会使视点太奇怪
		//else if (h<-1.0f) h = -1.0f;
		oldmx = x, oldmy = y; //把此时的鼠标坐标作为旧值，为下一次计算增量做准备
	}
}*/
void CPointCloudMatchinDlg::OnMButtonDown(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值

	CDialog::OnMButtonDown(nFlags, point);
	manager->getInitPos(point.x, point.y);
	midButtonState = true;
}


void CPointCloudMatchinDlg::OnMButtonUp(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值

	CDialog::OnMButtonUp(nFlags, point);
	midButtonState = false;
}


void CPointCloudMatchinDlg::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值

	CDialog::OnMouseMove(nFlags, point);
	if (midButtonState)
	{
		if (ctrlKeyState)
		{
			manager->executeTranslateOperation(point.x, point.y);
		}
		else
		{
			manager->executeRotateOperation(point.x, point.y);
		}

		OnPaint();
	}
}


BOOL CPointCloudMatchinDlg::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO:  在此添加消息处理程序代码和/或调用默认值
	if (zDelta >= 0)
	{
		manager->executeScaleOperation(-0.1);
	}
	else
	{
		manager->executeScaleOperation(0.1);
	}
	OnPaint();

	return CDialog::OnMouseWheel(nFlags, zDelta, pt);
}

BOOL CPointCloudMatchinDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO:  在此添加专用代码和/或调用基类  
	if (pMsg->message == WM_KEYDOWN)
	{

		if (pMsg->wParam == VK_CONTROL)//直接用虚码代替就可以响应所指键  
		{
			ctrlKeyState = true;
		}

	}

	if (pMsg->message == WM_KEYUP)
	{

		if (pMsg->wParam == VK_CONTROL)//直接用虚码代替就可以响应所指键  
		{
			ctrlKeyState = false;
		}

	}
	return CDialog::PreTranslateMessage(pMsg);
}
/*


void Mouse(int button, int state, int x, int y) //处理鼠标点击
{
if (state == GLUT_DOWN) //第一次鼠标按下时,记录鼠标在窗口中的初始坐标
oldmx = x, oldmy = y;
}
void onMouseMove(int x, int y) //处理鼠标拖动
{
//printf("%d\n",du);
du += x - oldmx; //鼠标在窗口x轴方向上的增量加到视点绕y轴的角度上，这样就左右转了
h += 0.03f*(y - oldmy); //鼠标在窗口y轴方向上的改变加到视点的y坐标上，就上下转了
if (h>1.0f) h = 1.0f; //视点y坐标作一些限制，不会使视点太奇怪
else if (h<-1.0f) h = -1.0f;
oldmx = x, oldmy = y; //把此时的鼠标坐标作为旧值，为下一次计算增量做准备
}*/