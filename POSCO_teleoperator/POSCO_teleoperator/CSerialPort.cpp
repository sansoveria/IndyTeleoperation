#include "CSerialPort.h"
#include <iostream>

using namespace std;
extern BYTE bbuff[32];
extern char strings[18];

CSerialPort::CSerialPort()
{
}

CSerialPort::~CSerialPort()
{
}

bool CSerialPort::OpenPort(CString portname)
{
	CString port = "//./";
	port += portname;

	//m_hComm = CreateFile(L"//./" + portname, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0); //�ø��� ��Ʈ�� �����Ѵ�. 
	m_hComm = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	if (m_hComm == INVALID_HANDLE_VALUE)  //���������� ��Ʈ�� ���ȴ��� Ȯ��
	{
		return false;  //������ �ʾ��� ��� false ��ȯ
	}
	else
		return true;   //����� ������ ��� true ��ȯ
}




bool CSerialPort::ConfigurePort(DWORD BaudRate, BYTE ByteSize, DWORD fParity,
	BYTE Parity, BYTE StopBits)
{
	if ((m_bPortReady = GetCommState(m_hComm, &m_dcb)) == 0) //��Ʈ�� ���¸� Ȯ��. ���������� ������ �ʾ��� ��� false ��ȯ
	{
		printf("\nGetCommState Error\n");
		//"MessageBox(L, L"Error", MB_OK + MB_ICONERROR);  
		CloseHandle(m_hComm);
		return false;
	}
	//��Ʈ�� ���� �⺻���� ����
	m_dcb.BaudRate = BaudRate;
	m_dcb.ByteSize = ByteSize;
	m_dcb.Parity = Parity;
	m_dcb.StopBits = StopBits;
	m_dcb.fBinary = true;
	m_dcb.fDsrSensitivity = false;
	m_dcb.fParity = fParity;
	m_dcb.fOutX = false;
	m_dcb.fInX = false;
	m_dcb.fNull = false;
	m_dcb.fAbortOnError = true;
	m_dcb.fOutxCtsFlow = false;
	m_dcb.fOutxDsrFlow = false;
	m_dcb.fDtrControl = DTR_CONTROL_DISABLE;
	m_dcb.fDsrSensitivity = false;
	m_dcb.fRtsControl = RTS_CONTROL_DISABLE;
	m_dcb.fOutxCtsFlow = false;
	m_dcb.fOutxCtsFlow = false;

	m_bPortReady = SetCommState(m_hComm, &m_dcb);  //��Ʈ ���� Ȯ��

	if (m_bPortReady == 0)  //��Ʈ�� ���¸� Ȯ��. ������ ��� true ��ȯ �ƴ� ��� false ��ȯ
	{
		//MessageBox(L"SetCommState Error");  
		printf("SetCommState Error");
		CloseHandle(m_hComm);
		return false;
	}

	return true;
}






bool CSerialPort::SetCommunicationTimeouts(DWORD ReadIntervalTimeout,
	DWORD ReadTotalTimeoutMultiplier, DWORD ReadTotalTimeoutConstant,
	DWORD WriteTotalTimeoutMultiplier, DWORD WriteTotalTimeoutConstant) //��� ��Ʈ�� ���� Timeout ����
{
	if ((m_bPortReady = GetCommTimeouts(m_hComm, &m_CommTimeouts)) == 0)
		return false;

	m_CommTimeouts.ReadIntervalTimeout = ReadIntervalTimeout; //����Ҷ� �ѹ���Ʈ�� ���� �� ���� ����Ʈ�� ���۵ɶ������� �ð�
		//��ſ��� �����͸� ���� �� Timeout�� ����� �������� ���� ����
	m_CommTimeouts.ReadTotalTimeoutConstant = ReadTotalTimeoutConstant;
	m_CommTimeouts.ReadTotalTimeoutMultiplier = ReadTotalTimeoutMultiplier;
	//��ſ��� �����͸� ������ �� Timeout�� ����� �������� ���� ����
	//m_CommTimeouts.WriteTotalTimeoutConstant = WriteTotalTimeoutConstant;
	m_CommTimeouts.WriteTotalTimeoutConstant = 1;
	m_CommTimeouts.WriteTotalTimeoutMultiplier = WriteTotalTimeoutMultiplier;

	m_bPortReady = SetCommTimeouts(m_hComm, &m_CommTimeouts);  //��Ʈ ���� Ȯ��

	if (m_bPortReady == 0) //��Ʈ ���°� ���� ���� ��� false��ȯ. �ƴ� ��� true��ȯ
	{
		//MessageBox(L"StCommTimeouts function failed",L"Com Port Error",MB_OK+MB_ICONERROR);  
		printf("\nStCommTimeouts function failed\n");
		CloseHandle(m_hComm);
		return false;
	}

	return true;
}







bool CSerialPort::WriteByte(char bybyte)
{
	//iBytesWritten=0;
	m_iBytesWritten = 0;
	cout << bybyte << endl;

	if (WriteFile(m_hComm, &bybyte, 1, &m_iBytesWritten, NULL) == 0) //�Է¹��� ���� WriteFile�� ���� ��Ʈ�� �����Ѵ�.
		return false;
	else
		return true;
}





bool CSerialPort::WriteByte(char* str, UINT size)
{
	DWORD dwBytesTransferred = 0;

	//if (WriteFile(m_hComm, resp, size, &dwBytesTransferred, 0))
	//{
	//	if (dwBytesTransferred == size)
	//		return true;
	//}

	//return false;


	//char Str[] = "Example text testing WriteFile";
	if (WriteFile(m_hComm, str, size, &dwBytesTransferred, NULL)) {
		//cout << "Sending Bytes : " << dwBytesTransferred << endl;
		return true;
	}

}




bool CSerialPort::ReadByte(BYTE& resp)
{
	BYTE rx;
	resp = 0;

	DWORD dwBytesTransferred = 0;

	if (ReadFile(m_hComm, &rx, 1, &dwBytesTransferred, 0)) //��Ʈ�� �����ϴ� �����͸� ReadFile�� ���� 1����Ʈ�� �о�´�.
	{
		if (dwBytesTransferred == 1) //�����͸� �о���µ� �������� ���
		{
			resp = rx;  //resp�� �����͸� �����ϰ� true ��ȯ
			return true;
		}
	}

	return false; //�������� ��� false ��ȯ
}








bool CSerialPort::ReadByte(char* str, UINT size)
{
	DWORD dwBytesTransferred = 0;

	if (ReadFile(m_hComm, str, size, &dwBytesTransferred, 0)) {
		return true;
	}
}






void CSerialPort::ClosePort()
{
	CloseHandle(m_hComm); //��Ʈ�� �ݴ´�.
	return;
}


void CSerialPort::PurgePort()
{
	//CloseHandle(m_hComm); //��Ʈ�� �ݴ´�.


	PurgeComm(m_hComm,
		PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

	return;
}
