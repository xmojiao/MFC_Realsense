
// MFC_RealSenseDlg.h : 头文件
//

#pragma once

#include "afxwin.h"
#include "AlgorithmDetect.h"
#include "globalVar.h"

//#include "MatQueue.h"
using namespace std;


void threadProc(LPVOID IpParameter);
// CMFC_RealSenseDlg 对话框
class CMFC_RealSenseDlg : public CDialogEx
{
// 构造
public:
	CMFC_RealSenseDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_MFC_REALSENSE_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持
//public:
//	static void testFunc(int it);
//	int m_t;
// 实现
protected:
	HICON m_hIcon;
	
	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();

public:
	friend void threadProc(LPVOID IpParameter);
	bool ctrlThread = false;
	CButton mBtnStart;
	afx_msg void OnBnClickedCancel();
	void funcProc();

	rs2::pointcloud pc;
	rs2::points points;
	rs2::pipeline pipe;

	AlgorithmDetect algo;
	float mBarrierHeightThresh;
	float mSideThresh;
	float mDistanceThresh;
	int mDownSampleRate;
	int mDownSampleType;
	afx_msg void OnBnClickedRadio1();
	BOOL isShowCube;
	afx_msg void OnBnClickedCheckbox();
	BOOL isShowPlane;
	afx_msg void OnBnClickedCheckbox2();
	afx_msg void OnEnChangeEdit1();
	// 是否保存数据
	CButton IsSaveheight;
};
