#pragma once
#include <opencv2/opencv.hpp>      
#include <iostream>       
using namespace std;
using namespace cv;
class MatQueue
{
private:
	unsigned int m_size;
	int m_front;
	int m_rear;
	cv::Mat **matPtArr;
public:
	MatQueue(unsigned size)
		:m_size(size),
		m_front(0),
		m_rear(0)
	{
		matPtArr = new cv::Mat *[m_size];
		for (int i =0; i!=m_size; i++)
		{
			matPtArr[i] = NULL;
		}
	}
	~MatQueue()
	{
		for (int i = 0; i!= m_size; i++)
		{
			delete matPtArr[i];
			matPtArr[i] = NULL;
		}
		delete[] matPtArr;
		matPtArr = NULL;
	}
	bool isEmpty()
	{
		return m_front == m_rear;
	}
	bool isFull()
	{
		return m_front == (m_rear + 1) % m_size;
	}
	void push(cv::Mat ele)throw(bad_exception)
	{
		/*if (isFull())
			throw bad_exception();*/
		if (matPtArr[m_rear]!= NULL)
		{
			delete matPtArr[m_rear];
			matPtArr[m_rear] = NULL;
		}
		matPtArr[m_rear] = new cv::Mat();
		ele.copyTo(*(matPtArr[m_rear]));
		m_rear = (m_rear + 1) % m_size;
	}
	cv::Mat pop()throw(bad_exception)
	{
		if (isEmpty())
			throw bad_exception();
		cv::Mat tmp;
		matPtArr[m_front]->copyTo(tmp);
		if (matPtArr[m_front] != NULL)
		{
			delete matPtArr[m_front];
			matPtArr[m_front] = NULL;
		}
		m_front = (m_front + 1) % m_size;
		return tmp;
	}
};

