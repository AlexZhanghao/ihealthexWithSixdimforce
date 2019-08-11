#include "file_writer.h"

#include <windows.h>


#include <iomanip>

using std::string;

FileWriter::FileWriter(const char *filename) {
	file.open(filename);

}

void FileWriter::Initial() {
	string current_time = GetLocalTime();
	string file_name("d:\\ihealth_data\\");
	file_name += current_time;
	file_name += ".txt";
	file.open(file_name, std::ofstream::app);
}

void FileWriter::WriteHeader() {
	if (!file) {
		return;
	}
	file << std::setw(18) << "���" << std::setw(18) << "�ⲿʵ��λ��" << std::setw(18) << "�粿ʵ��λ��" 
		<< std::setw(18) << std::setw(18) << "�ⲿ����λ��" << std::setw(18) << "�粿����λ��" <<std::setw(18)
		<< "�ⲿ��������" << std::setw(18) << "�粿��������" << std::setw(18) << "�ⲿ��������" << std::setw(18) 
		<< "�粿��������" << std::setw(18) <<"�ⲿ����" << std::setw(18) << "�粿����" << "\n";
}

void FileWriter::WriteString(string s) {
	if (!file) {
		return;
	}
	file << std::setw(18) << s;
}

void FileWriter::CloseFile() {
	if (file) {
		file.close();
	}
}

string FileWriter::GetLocalTime() {
	std::string current_time("");
	SYSTEMTIME time;
	::GetLocalTime(&time);
	char buf[100];
	sprintf_s(buf, "%.4hd-%.2hd-%.2hd %.2hd-%.2hd-%.2hd", time.wYear, time.wMonth, time.wDay, time.wHour, time.wMinute, time.wSecond);
	current_time += buf;
	return current_time;
}