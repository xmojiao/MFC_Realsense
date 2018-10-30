#pragma once
#include <iostream>
using namespace std;
template <class T>
class CycleQueue
{
public:
	unsigned int m_size;
	int m_front;
	int m_rear;
	T* m_data;
	unsigned long long m_count;
public:
	CycleQueue(unsigned size)
		:m_size(size),
		m_front(0),
		m_rear(0),
		m_count(0)
	{
		m_data = new T[size];
	}
	~CycleQueue()
	{
		delete[] m_data;
		m_data = nullptr;
	}
	bool isEmpty()
	{
		return m_front == m_rear;
	}
	bool isFull()
	{
		return m_front == (m_rear + 1) % m_size;
	}
	bool isFulled()
	{
		return m_count >= m_size;

	}

	void push(T ele)/*throw(bad_exception)*/
	{
		if (isFull())	
			m_front = (m_front + 1) % m_size;
			/*throw bad_exception();*/
		m_data[m_rear] = ele;
		m_rear = (m_rear + 1) % m_size;
		m_count++;
	}
	T pop()/*throw(bad_exception)*/
	{
		if (isEmpty())
			return NULL;
		T tmp = m_data[m_front];
		m_front = (m_front + 1) % m_size;
		return tmp;
	}
};

/*try {
m_queue.push(points);
}
catch (const std::bad_exception& e)
{
std::cerr << "Caught " << e.what() << '\n';
}
cout << "the number of frame: " << points.get_frame_number() << endl;
cout << "Point number is: " << points.size() << endl;

if (m_queue.isFulled())
{
rs2::points tmp;
try {
tmp = m_queue.pop();
}
catch (const std::bad_exception& e)
{
std::cerr << "Caught " << e.what() << '\n';
}
cout << "the number of pop points: " << tmp.size() << endl;
cout << "the frame number of pop points is : "<<tmp.get_frame_number() << endl;
}
*/

