
// MFC_RealSenseDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "MFC_RealSense.h"
#include "MFC_RealSenseDlg.h"
#include "afxdialogex.h"

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


// CMFC_RealSenseDlg 对话框



CMFC_RealSenseDlg::CMFC_RealSenseDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_MFC_REALSENSE_DIALOG, pParent)
	, mBarrierHeightThresh(30)
	, mSideThresh(600)
	, mDistanceThresh(2000)
	, mDownSampleType(0)
	, mDownSampleRate(4)
	, isShowCube(FALSE)
	, isShowPlane(FALSE)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);	
}

void CMFC_RealSenseDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDOK, mBtnStart);
	DDX_Text(pDX, IDC_EDIT1, mBarrierHeightThresh);
	DDX_Text(pDX, IDC_EDIT2, mSideThresh);
	DDX_Text(pDX, IDC_EDIT3, mDistanceThresh);
	DDX_Radio(pDX, IDC_RADIO1, mDownSampleType);
	DDX_Check(pDX, IDC_CHECKBOX, isShowCube);
	DDX_Check(pDX, IDC_CHECKBOX2, isShowPlane);
	DDX_Control(pDX, IDCANCEL, IsSaveheight);
}

BEGIN_MESSAGE_MAP(CMFC_RealSenseDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDOK, &CMFC_RealSenseDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &CMFC_RealSenseDlg::OnBnClickedCancel)
	
	ON_BN_CLICKED(IDC_RADIO1, &CMFC_RealSenseDlg::OnBnClickedRadio1)
	ON_BN_CLICKED(IDC_RADIO2, &CMFC_RealSenseDlg::OnBnClickedRadio1)
	ON_BN_CLICKED(IDC_RADIO3, &CMFC_RealSenseDlg::OnBnClickedRadio1)
	ON_BN_CLICKED(IDC_CHECKBOX, &CMFC_RealSenseDlg::OnBnClickedCheckbox)
	ON_BN_CLICKED(IDC_CHECKBOX2, &CMFC_RealSenseDlg::OnBnClickedCheckbox2)
END_MESSAGE_MAP()


// CMFC_RealSenseDlg 消息处理程序

BOOL CMFC_RealSenseDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
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

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CMFC_RealSenseDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CMFC_RealSenseDlg::OnPaint()
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

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CMFC_RealSenseDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CMFC_RealSenseDlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData();
	algo.mDistanceThresh = mDistanceThresh / 1000.0f;
	algo.mSideThresh = mSideThresh / 1000.0f;
	algo.mBarrierHeightThresh = mBarrierHeightThresh / 1000.0f;
	algo.mDownSampleRate = mDownSampleRate;
	algo.showBox = isShowCube;
	algo.showPlane = isShowPlane;
	if (!algoCtrl)
	{	
		algoCtrl = !algoCtrl;
		mBtnStart.SetWindowText(_T("停止检测"));
		AfxBeginThread((AFX_THREADPROC)threadProc, (LPVOID)this);  //创建线程
	}
	else
	{		
		algoCtrl = !algoCtrl;
		mBtnStart.SetWindowText(_T("开始检测"));
	}	
}

void threadProc(LPVOID IpParamater)
{
	CMFC_RealSenseDlg *p = (CMFC_RealSenseDlg*)IpParamater;
	p->funcProc();
}


void CMFC_RealSenseDlg::funcProc()
{
	algo.startDetectFromFile();
	mBtnStart.SetWindowText(_T("开始检测"));
}


void CMFC_RealSenseDlg::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	if (algoCtrl)
	{
		algoCtrl = !algoCtrl;
		mBtnStart.SetWindowText(_T("开始检测"));
		Sleep(500);
	}	
	CDialogEx::OnCancel();
}

void CMFC_RealSenseDlg::OnBnClickedRadio1()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(true);
	switch (mDownSampleType)
	{
	case 0:
		mDownSampleRate = 4;
		break;
	case 1:
		mDownSampleRate = 2;
		break;
	case 2:
		mDownSampleRate = 1;
		break;
	default:
		break;
	}
}


void CMFC_RealSenseDlg::OnBnClickedCheckbox()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(true);
}


void CMFC_RealSenseDlg::OnBnClickedCheckbox2()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(true);
}
