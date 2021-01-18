
// MFCApplication1Dlg.h: 헤더 파일
//

#pragma once
// header for TwinCAT ADS communication
#include "C:\TwinCAT\AdsApi\TcAdsDll\Include\TcAdsDef.h"
#include "C:\TwinCAT\AdsApi\TcAdsDll\Include\TcAdsAPI.h"
#include <iostream>

using namespace std;

// CMFCApplication1Dlg 대화 상자
class CMFCApplication1Dlg : public CDialogEx
{
// 생성입니다.
public:
	CMFCApplication1Dlg(CWnd* pParent = nullptr);	// 표준 생성자입니다.

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_MFCAPPLICATION1_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.


// 구현입니다.
private:
	// TwinCAT ADS communication
	long						nErr;						// Error Message
	long						nPort;						// Port number
	AmsAddr						Addr;						// Target port information(PLC1)

protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedMotorEnable();
	afx_msg void OnBnClickedMotorDisable();
	afx_msg void OnBnClickedControlOn();
	afx_msg void OnBnClickedControlOff();
	afx_msg void OnBnClickedControlZero();


	CButton button_motor_enable;
	CButton button_motor_disable;
	CButton button_control_on;
	CButton button_control_off;
	
#pragma pack(push, 1)
	struct ADS_INPUT{
		// variable order and size are important!
		// TC INT == SHORT
		// TC DINT == LONG / INT
		// TC BOOL == bool
		INT u1;
		INT v1;
		INT w1;
		INT u2;
		INT v2;
		INT w2;
		INT angle1;
		INT angle2;
		INT angleZero1;
		INT angleZero2;
		bool motorState1;
		bool motorState2;
	};

	struct ADS_OUTPUT {
		// variable order and size are important!
		// TC INT == SHORT
		// TC DINT == LONG / INT
		// TC BOOL == bool
		INT tau1;
		INT tau2;
		INT tau3;
		bool enableMotor1;
		bool enableMotor2;
		bool calibrationFlag;
	};
#pragma pack(pop)

	ADS_INPUT aInput;
	ADS_OUTPUT aOutput;
};
