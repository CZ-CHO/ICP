#include "FileIO.h"

//����ָ�룬���ڴ�ŵ�����
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);    //��׼����
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);    //��׼����
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);    //�洢��׼�����ļ����������ݣ�������ֵ9999���ݣ������ڽ�����תƽ�Ʊ任
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);    //�洢cloud3�ı任���

FileIO::FileIO()
{
}

FileIO::~FileIO()
{
}

void FileIO::readASC(const char *file1, const char *file2)
{
	ifstream infile1, infile2;
	infile1.open(file1);   //�������ļ�
	infile2.open(file2);   //�������ļ� 
	string s1,s2;    //��������ļ�������
	int windowsize = 1;    //�¶ȹ��˴��ڰ뾶
	string str1[4];    //�洢���кż�դ���������

	//��ȡasc�ļ�ͷ
	for (int i = 0; i < 6; i++)
	{
		getline(infile1, s1);
		if (i < 4)    //�����������ļ�ͷǰ4��
		{
			char *p = strtok((char*)s1.data(), " ");    //���ַ���s�Կո�ָ�
			p = std::strtok(NULL, " ");    //��ȡÿ�еڶ�������
			str1[i] = p;
		}
	}

	cloud1->width = atoi(str1[0].c_str());    //դ����
	cloud1->height = atoi(str1[1].c_str());    //դ��߶�
	//int x1 = round(atof(str1[2].c_str()) * 10000);
	//int y1 = round(atof(str1[3].c_str()) * 10000);
	float x1 = atof(str1[2].c_str());    //դ�����x����
	float y1 = atof(str1[3].c_str());    //դ�����y����
	cloud1->is_dense = false;
	cloud1->points.resize(cloud1->width * cloud1->height);


	/*********************************���¶ȹ���**********************************/
	float **t = new float*[cloud1->height];    //�����д��ԭʼ����zֵ
	float **g = new float*[cloud1->height];    //���ڴ���¶ȹ��˺�����
	for (int i = 0; i < cloud1->height; i++)    
	{
		t[i] = new float[cloud1->width];
		g[i] = new float[cloud1->width];

		getline(infile1, s1);
		char *p = strtok((char*)s1.data(), " ");    //���ַ���s�Կո�ָ�
		for (int j = 0; j < cloud1->width; j++)
		{
			t[i][j] = atof(p);
			g[i][j] = atof(p);
			p = std::strtok(NULL, " ");
		}
	}
	int a = 0;    //��������׼�ĸ̵߳���м���
	for (int i = 0; i < cloud1->height; i++)
	{
		for (int j = 0; j < cloud1->width; j++)
		{
			if (t[i][j] > 1000) continue;    //����ȡ��ֵ�㣨��asc�ļ���Ϊ9999��    

			//���߲�仯��ĸ̵߳���ȥ
			float tmax = t[i][j];
			float tmin = t[i][j];
			for (int c = -windowsize; c <= windowsize; c++)
			{
				if (i + c < 0 || i + c >= cloud1->height) continue;    //����
				for (int d = -windowsize; d <= windowsize; d++)
				{
					if (j + d < 0 || j + d >= cloud1->width) continue;    //����
					if (t[i + c][j + d] > 1000) continue;    //��ֵ��

					tmax = tmax > t[i + c][j + d] ? tmax : t[i + c][j + d];
					tmin = tmin < t[i + c][j + d] ? tmin : t[i + c][j + d];
				}
			}
			if (tmax - tmin > 2)    //����߲�����򽫸õ��ÿ�
			{
				g[i][j] = 9999;
				continue;
			}

			//����׼���ƣ��������Ļ���������ʼ����Ϊ(0,0)
			cloud1->points[a].x = j;    
			cloud1->points[a].y = -i;    //y������ϵ��µݼ�
			cloud1->points[a].z = t[i][j];
			a++;
		}
	}
	cloud1->points.resize(a);    //���õ���


	/*********************************δ���¶ȹ���**********************************/
	//int a = 0;
	//while (getline(infile1, s1))    //ÿ�ζ�ȡһ�����ݣ������ַ�������s��
	//{
	//	float j = x1;
	//	//string b = std::to_string(y1);    //ÿ��ѭ������yֵ��������ת��Ϊ�ַ���
	//	char *p = strtok((char*)s1.data(), " ");    //���ַ���s�Կո�ָ�
	//	while (p != 0)
	//	{
	//		if (strcmp(p, "9999"))    //�ַ����Ƚϣ�������򷵻�0
	//		{
	//			cloud1->points[a].x = j;
	//			cloud1->points[a].y = y1;
	//			cloud1->points[a].z = atof(p);
	//			a++;
	//		}
	//		p = std::strtok(NULL, " ");
	//		j += 10000;
	//	}
	//	y1 -= 10000;
	//}
	//cloud1->points.resize(a);

	infile1.close();    //�ر��ļ�������


	string str2[4];    //�洢���кż�դ���������
	for (int i = 0; i < 6; i++)
	{
		getline(infile2, s2);
		if (i < 4)    //�����������ļ�ͷ3��4��
		{
			char *p = strtok((char*)s2.data(), " ");    //���ַ���s�Կո�ָ�
			p = std::strtok(NULL, " ");    //��ȡ�ڶ�������
			str2[i] = p;
		}
	}

	cloud2->width = atoi(str2[0].c_str());
	cloud2->height = atoi(str2[1].c_str());
	cloud2->is_dense = false;
	cloud2->points.resize(cloud2->width * cloud2->height);
	cloud3->width = atoi(str2[0].c_str());
	cloud3->height = atoi(str2[1].c_str());
	cloud3->is_dense = false;
	cloud3->points.resize(cloud3->width * cloud3->height);
	cloud4->width = atoi(str2[0].c_str());
	cloud4->height = atoi(str2[1].c_str());
	cloud4->is_dense = false;
	cloud4->points.resize(cloud3->width * cloud3->height);

	//int x2 = round(atof(str2[2].c_str()) * 10000);
	//int y2 = round(atof(str2[3].c_str()) * 10000);
	float x2 = atof(str2[2].c_str()) - x1;    //դ�����x���꣨���Ļ���
	float y2 = atof(str2[3].c_str()) - y1;    //դ�����y���꣨���Ļ���



	/*********************************���¶ȹ���**********************************/
	int b = 0;    //��������׼�ĸ̵߳���м���
	int f = 0;    //�ܵ�������
	float **q = new float*[cloud2->height];    //�洢����zֵ
	float **h = new float*[cloud2->height];    //�洢�¶ȹ��˺�ĵ�zֵ
	for (int i = 0; i < cloud2->height; i++)
	{
		q[i] = new float[cloud2->width];
		h[i] = new float[cloud2->width];
		getline(infile2, s2);
		char *p = strtok((char*)s2.data(), " ");    //���ַ���s�Կո�ָ�
		for (int j = 0; j < cloud2->width; j++)
		{
			cloud3->points[f].x = x2 + j;
			cloud3->points[f].y = y2 - i;
			cloud3->points[f].z = atof(p);
			f++;

			q[i][j] = atof(p);
			h[i][j] = atof(p);
			p = std::strtok(NULL, " ");
		}
	}
	for (int i = 0; i < cloud2->height; i++)
	{
		for (int j = 0; j < cloud2->width; j++)
		{
			if (q[i][j] > 1000) continue;

			float tmax = q[i][j];
			float tmin = q[i][j];
			for (int c = -windowsize; c <= windowsize; c++)
			{
				if (i + c < 0 || i + c >= cloud2->height) continue;
				for (int d = -windowsize; d <= windowsize; d++)
				{
					if (j + d < 0 || j + d >= cloud2->width) continue;
					if (q[i + c][j + d] > 1000) continue;

					tmax = tmax > q[i + c][j + d] ? tmax : q[i + c][j + d];
					tmin = tmin < q[i + c][j + d] ? tmin : q[i + c][j + d];
				}
			}
			if (tmax - tmin > 2)
			{
				h[i][j] = 9999;
				continue;
			}

			cloud2->points[b].x = x2 + j;
			cloud2->points[b].y = y2 - i;
			cloud2->points[b].z = q[i][j];
			b++;
		}
	}
	cloud2->points.resize(b);    //���õ���
	infile2.close();    //�ر��ļ�������


	/*********************************δ���¶ȹ���**********************************/
	//int b = 0;
	//int m = 0;
	//while (getline(infile2, s2))    //ÿ�ζ�ȡһ�����ݣ������ַ�������s��
	//{
	//	float j = x2;
	//	//string b = std::to_string(y2);    //ÿ��ѭ������yֵ��������ת��Ϊ�ַ���
	//	char *p = strtok((char*)s2.data(), " ");    //���ַ���s�Կո�ָ�
	//	while (p != 0)
	//	{
	//		if (strcmp(p, "9999"))    //�ַ����Ƚϣ�������򷵻�0
	//		{
	//			cloud2->points[b].x = j;
	//			cloud2->points[b].y = y2;
	//			cloud2->points[b].z = atof(p);
	//			b++;

	//			/*cloud3->points[m].x = j;
	//			cloud3->points[m].y = y2;
	//			cloud3->points[m].z = atof(p);
	//			m++;*/
	//		}
	//		//else
	//		//{
	//			cloud3->points[m].x = j;
	//			cloud3->points[m].y = y2;
	//			cloud3->points[m].z = atof(p);
	//			m++;
	//		//}

	//		p = std::strtok(NULL, " ");
	//		j += 10000;
	//	}
	//	y2 -= 10000;
	//}
	//cloud2->points.resize(b);
	

	std::cout << "Start registration" << endl;
	std::cout << "pointcloud1 point num: " << a << endl;    //������˺����
	std::cout << "pointcloud2 point num: " << b << endl;
	int iterations = 100;    //icp��������
	int maxdistance = 0;   //������

	// Defining a rotation matrix and translation vector
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations(iterations);
	//icp.setMaxCorrespondenceDistance(maxdistance);
	
	icp.setInputSource(cloud1);
	icp.setInputTarget(cloud2);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);    //ִ��icp

	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP iteration " << iterations << std::endl;
		std::cout << "\nICP max distance " << maxdistance << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();    //��ȡ�任����
		print4x4Matrix(transformation_matrix);
		pcl::transformPointCloud(*cloud3, *cloud4, transformation_matrix);    //Ӧ�ñ任
		std::cout << cloud3->points[0].x << " " << cloud3->points[0].y << " " 
			<< cloud4->points[0].x << " " << cloud4->points[0].y << std::endl;    //����任ǰ��ĵ���������꣨���ڼ�飩

		string outf1 = "C:\\Users\\564146665\\Desktop\\data\\result\\transformData\\2-3_" + 
			to_string(windowsize) + "_" + to_string(iterations) + "_" + to_string(maxdistance) + ".asc";
		ofstream outfile(outf1);    //������ļ�

		//����ļ�ͷ��Ϣ
		outfile << "ncols " << cloud2->width << endl;
		outfile << "nrows " << cloud2->height << endl;
		outfile << "xllcorner " << fixed << setprecision(4) << cloud4->points[0].x + x1 << endl;
		outfile << "yllcorner " << fixed << setprecision(4) << cloud4->points[0].y + y1 << endl;
		outfile << "cellsize " << "1" << endl;
		outfile << "nodata_value " << "9999" << endl;

		//����߳�ֵ
		for (int i = 0; i < cloud4->height; i++)
		{
			for (int j = 0; j < cloud4->width; j++)
			{
				if(cloud4->points[i*cloud4->width + j].z > 1000)
					outfile << "9999 ";
				else
				    outfile << fixed << setprecision(10) << cloud4->points[i*cloud4->width + j].z << " ";
			}
			outfile << endl;
		}
		outfile.close();

		//���cloud1�¶ȹ��˽��
		ofstream outfile1("C:\\Users\\564146665\\Desktop\\data\\result\\slopeData\\2_2_3.asc");
		outfile1 << "ncols " << cloud1->width << endl;
		outfile1 << "nrows " << cloud1->height << endl;
		outfile1 << "xllcorner " << str1[2] << endl;
		outfile1 << "yllcorner " << str1[3] << endl;
		outfile1 << "cellsize " << "1" << endl;
		outfile1 << "nodata_value " << "9999" << endl;
		for (int i = 0; i < cloud1->height; i++)
		{
			for (int j = 0; j < cloud1->width; j++)
			{
				if (g[i][j] > 1000)
					outfile1 << "9999 ";
				else
					outfile1 << fixed << setprecision(10) << g[i][j] << " ";
			}
			outfile1 << endl;
		}
		outfile1.close();

		//���cloud2�¶ȹ��˽��
		ofstream outfile2("C:\\Users\\564146665\\Desktop\\data\\result\\slopeData\\3_2_3.asc");    //������ļ�
		outfile2 << "ncols " << cloud2->width << endl;
		outfile2 << "nrows " << cloud2->height << endl;
		outfile2 << "xllcorner " << str2[2] << endl;
		outfile2 << "yllcorner " << str2[3] << endl;
		outfile2 << "cellsize " << "1" << endl;
		outfile2 << "nodata_value " << "9999" << endl;
		for (int i = 0; i < cloud2->height; i++)
		{
			for (int j = 0; j < cloud2->width; j++)
			{
				if (h[i][j] > 1000)
					outfile2 << "9999 ";
				else
					outfile2 << fixed << setprecision(10) << h[i][j] << " ";
			}
			outfile2 << endl;
		}
		outfile2.close();
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
	}
}

void FileIO::print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("\n Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("\n Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
	//printf("%6.3f, %6.3f, %6.3f, %6.3f >\n", matrix(3, 0), matrix(3, 1), matrix(3, 2), matrix(3, 3));
}
