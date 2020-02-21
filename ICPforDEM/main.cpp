#include <stdio.h>
#include <iostream>
#include "FileIO.h"
using namespace std;

int main()
{
	FileIO f;
	const char *file1 = "C:\\Users\\564146665\\Desktop\\data\\result\\interpolateData\\2_Ini_NatN.asc";
	const char *file2 = "C:\\Users\\564146665\\Desktop\\data\\result\\interpolateData\\3_Ini_NatN.asc";
	//const char *file1 = "C:\\Users\\564146665\\Desktop\\data\\result\\filterData\\1_Ini_Ground.asc";
	//const char *file2 = "C:\\Users\\564146665\\Desktop\\data\\result\\filterData\\2_Ini_Ground.asc";
	f.readASC(file1, file2);
	system("pause");
}