
// MFCApplication1Dlg.cpp: 구현 파일
//

#include "pch.h"
#include "framework.h"
#include "MFCApplication1.h"
#include "MFCApplication1Dlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

// 구현입니다.
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


// CMFCApplication1Dlg 대화 상자



CMFCApplication1Dlg::CMFCApplication1Dlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_MFCAPPLICATION1_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CMFCApplication1Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_MOTOR_ENABLE, button_motor_enable);
	DDX_Control(pDX, IDC_MOTOR_DISABLE, button_motor_disable);
	DDX_Control(pDX, IDC_CONTROL_ON, button_control_on);
	DDX_Control(pDX, IDC_CONTROL_OFF, button_control_off);
}

BEGIN_MESSAGE_MAP(CMFCApplication1Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_MOTOR_ENABLE, &CMFCApplication1Dlg::OnBnClickedMotorEnable)
	ON_BN_CLICKED(IDC_MOTOR_DISABLE, &CMFCApplication1Dlg::OnBnClickedMotorDisable)
	ON_BN_CLICKED(IDC_CONTROL_ON, &CMFCApplication1Dlg::OnBnClickedControlOn)
	ON_BN_CLICKED(IDC_CONTROL_OFF, &CMFCApplication1Dlg::OnBnClickedControlOff)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_CONTROL_ZERO, &CMFCApplication1Dlg::OnBnClickedControlZero)
END_MESSAGE_MAP()


// CMFCApplication1Dlg 메시지 처리기

BOOL CMFCApplication1Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
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

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	button_motor_disable.EnableWindow(FALSE);
	button_control_on.EnableWindow(FALSE);
	button_control_off.EnableWindow(FALSE);

	// ADS router에 ADS Port Open 
	nPort = AdsPortOpen();
	nErr = AdsGetLocalAddress(&Addr);
	if (nErr) {
		cerr << "[TC:ADS]Error: AdsGetLocalAddress: " << nErr << "\n";
	}
	else cerr << "[TC:ADS] ADS Port Opened\n";

	// TwinCAT 3 Module Port = 350
	(&Addr)->port = 350;
	// TODO: 여기에 추가 초기화 작업을 추가합니다.

	aOutput.enableMotor1 = false;
	aOutput.enableMotor2 = false;
	aOutput.calibrationFlag = false;

	//write
	USHORT initEPOS;
	initEPOS = 0x0006;
	nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x81000000, sizeof(initEPOS), &initEPOS);
	if (nErr != 0) {
		cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	}


	SetTimer(1, 100, NULL);
	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CMFCApplication1Dlg::OnTimer(UINT_PTR nIDEvent)
{
	CDialogEx::OnTimer(nIDEvent);

	switch (nIDEvent) {
	case 1:
		//std::cout << sizeof(ADS_INPUT) << ", " << sizeof(ADS_OUTPUT) << ", " << &aOutput << ", " << &aOutput.enableMotor1 << ", " << &aOutput.enableMotor2 << ", " << &aOutput.doCalibration << ", " << &aOutput.tau1 << ", " << &aOutput.tau2 << endl;

		nErr = AdsSyncReadReq(&Addr, 0x1010010, 0x82000000, sizeof(ADS_INPUT), &aInput);
		if (nErr != 0) {
			cerr << "[TC:ADS]Error: AdsSyncReadReq Failed: " << nErr << "\n";
		}
		nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000000, sizeof(ADS_OUTPUT), &aOutput);
		if (nErr != 0) {
			cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
		}
		break;
	}

}

void CMFCApplication1Dlg::OnSysCommand(UINT nID, LPARAM lParam)
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

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 애플리케이션의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CMFCApplication1Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CMFCApplication1Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CMFCApplication1Dlg::OnBnClickedMotorEnable()
{
	button_motor_enable.EnableWindow(FALSE);
	button_motor_disable.EnableWindow(TRUE);
	button_control_on.EnableWindow(TRUE);
	button_control_off.EnableWindow(FALSE);

	aOutput.enableMotor1 = true;
	aOutput.enableMotor2 = true;

	//USHORT initEPOS;
	//initEPOS = 0x0F;
	//nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x81000000, sizeof(initEPOS), &initEPOS);
	//if (nErr != 0) {
	//	cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	//}
	//nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x81000004, sizeof(initEPOS), &initEPOS);
	//if (nErr != 0) {
	//	cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	//}

	//AdsSyncReadWriteReq()
}


void CMFCApplication1Dlg::OnBnClickedMotorDisable()
{
	button_motor_enable.EnableWindow(TRUE);
	button_motor_disable.EnableWindow(FALSE);
	button_control_on.EnableWindow(FALSE);
	button_control_off.EnableWindow(FALSE);

	aOutput.enableMotor1 = false;
	aOutput.enableMotor2 = false;

	//USHORT initEPOS;
	//initEPOS = 0x0006;
	//nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x81000000, sizeof(initEPOS), &initEPOS);
	//if (nErr != 0) {
	//	cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	//}
	//nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x81000004, sizeof(initEPOS), &initEPOS);
	//if (nErr != 0) {
	//	cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	//}
}


void CMFCApplication1Dlg::OnBnClickedControlOn()
{
	button_control_on.EnableWindow(FALSE);
	button_control_off.EnableWindow(TRUE);

	float torque1, torque2, torque3;
	torque1 = 0.0;
	torque2 = float(-0.005);
	torque3 = 0.0;

	aOutput.tau1 = INT(torque1 * 1000);
	aOutput.tau2 = INT(torque2 * 1000);
	aOutput.tau3 = INT(torque3 * 1000);

	//INT control1, control2, control3;
	//control1 = torque1 * 1000;
	//control2 = torque2 * 1000;
	//control3 = torque3 * 1000;

	//nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000001, sizeof(control1), &control1);
	//if (nErr != 0) {
	//	cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	//}
	//nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000005, sizeof(control2), &control2);
	//if (nErr != 0) {
	//	cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	//}
	//nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000009, sizeof(control3), &control3);
	//if (nErr != 0) {
	//	cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	//}
}


void CMFCApplication1Dlg::OnBnClickedControlOff()
{
	button_control_on.EnableWindow(TRUE);
	button_control_off.EnableWindow(FALSE);

	float torque1, torque2, torque3;
	torque1 = 0.0;
	torque2 = 0.0;
	torque3 = 0.0;

	aOutput.tau1 = INT(torque1 * 1000);
	aOutput.tau2 = INT(torque2 * 1000);
	aOutput.tau3 = INT(torque3 * 1000);

	//SHORT control = 0;
	//nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x81000002, sizeof(control), &control);
	//if (nErr != 0) {
	//	cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	//}
	//nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x81000006, sizeof(control), &control);
	//if (nErr != 0) {
	//	cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	//}
}


void CMFCApplication1Dlg::OnBnClickedControlZero()
{
	button_control_on.EnableWindow(TRUE);
	button_control_off.EnableWindow(FALSE);

	if (aOutput.calibrationFlag) aOutput.calibrationFlag = false;
	else aOutput.calibrationFlag = true;

	//bool zero = true;
	//nErr = AdsSyncWriteReq(&Addr, 0x1010010, 0x83000000, sizeof(zero), &zero);
	//if (nErr != 0) {
	//	cerr << "[TC:ADS]Error: AdsSyncWriteReq Failed: " << nErr << "\n";
	//}
}
