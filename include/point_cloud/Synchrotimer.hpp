#pragma once
#include <common.hpp>
#include <point_cloud/common.h>

class Synchrotimer
{
public:
	void setStartFrameID(int value) { startFrameID = value;}
	/**
	 * 获取雷达帧(第一个点)的时间戳, 以秒为单位
	 */
	double getTime(PointCloud::Ptr frame)
	{
		return frame->points[0].data_n[0] + frame->points[0].data_n[1];
	}

	/**
	 * 获取雷达帧(最后一个点)的时间戳, 以秒为单位
	 */
	double getEndTime(PointCloud::Ptr frame)
	{
		return frame->points.back().data_n[0] + frame->points.back().data_n[1];
	}

	/**
	 * 使两个雷达帧的时间差小于半帧时间，纠正的是 pcap1 的 frameID
	 * 有三种返回的结果:
	 * [-1]: pcap2无效，跳过它
	 * [ 0]: 已纠正pcap1的frameID何pcap2的frameOffset, 请重新读取
	 * [ 1]: 无需纠正，可以合并  
	 */
	int correctFrameOffset(PointCloud::Ptr frame1,
		PointCloud::Ptr frame2, long long &frameID, int &offset)
	{
		//202也可能掉帧，判断一下
		float frameTime = getEndTime(frame2) - getTime(frame2);

		if (frame2->points.size() < 8000 || frameTime > 0.06)
		{
			std::cout << "Attention: 202可能存在掉帧或者数据不稳定情况!!!\n";
			return -1; //不纠正
		}

		// 以第一个点时间戳作为当前帧时间戳
		float timeOffset = getTime(frame2) - getTime(frame1);
		frameTime = 0.05; //正确的时间差

		//计算两个激光头之间的对应帧号差
		int _a = timeOffset / frameTime;
		// 让时间差位于半帧的时间之内
		_a += 1.9 * (timeOffset - _a * frameTime) / frameTime;
		offset -= _a;


		if(offset != 0)
		{
			std::cout << _a << std::endl;
			std::cout << "======================offset============================\n"
						  << "FrameID = " << frameID << "\t"
						  << "Offset = " << -_a << "\t"
						  << "Sumoffset = " << offset << "\t "
						  << "Timeoffset  = " << timeOffset << std::endl;
				//showTime(frame1, frameID, "202");
				//showTime(frame2, reader2.frameNumber - skipFrameNumber, "201");
				std::cout << "=============================================================\n";
		}
		
		if (abs(_a) > 0)
		{
			if (abs(_a) > 1 && frameID - startFrameID > 5)
				{
					std::cout << "\nAttention: Frame2's timestamp is changing more than 0.25s!!!\n";
					return -1;
				}


			if (frameID + offset < 3)
			{
				frameID += _a;
				#if DEBUG
				std::cout << "Change ID of frame1 to " << frameID << std::endl;
				#endif
			}
			return 0;
		}
		return 1;
	}
private:
	int startFrameID = 0;
};