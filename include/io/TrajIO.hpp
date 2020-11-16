#pragma once
#include <utility>
#include <time.h>
#include <iomanip>
#include <map>
#include <common.hpp>
#include <point_cloud/common.hpp>
#include <math/TransformTool.hpp>
#include <io/FileOperator.hpp>

enum TrajType{
    ROS_LOAM, G2OT, G2O, KITTI
};

class PoseNode
{
public:
    long long frameID;
    Eigen::Matrix4d tf;
    double timestamp;
    Eigen::Quaterniond qua;
    Eigen::Vector3d pos;
    Eigen::Matrix<double, 6, 1> loamTrans;

    PoseNode() : frameID(0), timestamp(0), //label(false),
                 tf(Eigen::Matrix4d::Identity())
    {
        //loamTrans2.resize(12);
    }

    PoseNode operator=(const PoseNode &value)
    {
         std::cout << "assign to this->tf" << std::endl;
        this->frameID = value.frameID;
        this->timestamp = value.timestamp;
        //this->label = value.label;
       
        this->tf = value.tf;
        std::cout << "assign successfully." << std::endl; 
        return *this;
    }
};

class TrajIO
{
    using string = std::string;
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    template <typename T> using vector = std::vector<T>;

private:
    std::map<TrajType, string> traj_format = {
        {ROS_LOAM, "| ROS_LOAM FrameID rx ry rz tx ty tz timestamp |"},
        {G2OT, "| iSAM_VERTEX FrameID x y z qx qy qz qw timestamp |"},
        {G2O, "| VERTEX_SE3:QUAT FrameID x y z qx qy qz qw | "}
    };

public:
    TrajIO(std::string traj_path)
    :   startID(-1),
        endID(-1),
        frameGap(0),
        data_name(fop.getFileName(traj_path)),
        pose_cnt(0)
    {
        if(!fop.isExist(traj_path))
        {
            std::cout << "Cannot open trajectory file: " << traj_path << "!" << std::endl;
            exit(-1);
        } 

        traj_file_.open(traj_path.c_str());

        string line;
        getline(traj_file_, line); // 获取轨迹文件的格式

        vector<string> st;
        boost::trim(line);
        boost::split(st, line, boost::is_any_of(" \t\n") , boost::token_compress_on);

        if(st[0] == "ROS_LOAM") traj_type = ROS_LOAM;
        else if(st[0] == "iSAM_VERTEX") traj_type = G2OT;
        else if(st[0] == "VERTEX_SE3:QUAT") traj_type = G2O;
        else traj_type = KITTI;

        startID = atoi(st[1].c_str());

        switch(traj_type)
        {
            case ROS_LOAM:
            {
                curPose.frameID = atoi(st[1].c_str());
                for (int i = 0; i < 6; ++i) curPose.loamTrans[i] = atof(st[2+i].c_str());
                curPose.timestamp = atof(st[8].c_str());
                curPose.tf = tt.euler2matrix(curPose.loamTrans);
                
                traj.insert( std::make_pair(curPose.frameID, curPose) );
                break;
            }
            case G2OT:
            {
                // 9 number each line: {Frame Number, Position[x|y|z], Quaternion[qx|qy|qz|qw], Timestamp(us)}
                curPose.frameID = atoi(st[1].c_str());
                for(int i=0; i<7; ++i) tq[i] = atof(st[2+i].c_str());
                curPose.tf = tt.tq2matrix(tq);
                curPose.timestamp = atof(st[9].c_str());
                
                traj.insert( std::make_pair(curPose.frameID, curPose) );
                break;
            }
            case G2O:
            {
                // 8 number each line: {Frame Number, Position[x|y|z], Quaternion[qx|qy|qz|qw]}
                curPose.frameID = atoi(st[1].c_str());
                for(int i=0; i<7; ++i) tq[i] = atof(st[2+i].c_str());
                curPose.tf = tt.tq2matrix(tq);
            
                traj.insert( std::make_pair(curPose.frameID, curPose) );
                break;
            }
            case KITTI:
            {
                curPose.tf = Eigen::Matrix4d::Identity();
        
                for(int row=0; row<3; ++row)
                    for(int col=0; col<4; ++col)
                        curPose.tf(row, col) = (double)atof(st[row*4+col].c_str());
                
                curPose.frameID = (long long)pose_cnt++;
                traj.insert( std::make_pair(curPose.frameID, curPose) );
                break;
            }
        }

        run();
    }

    void run()
    {
        // 获取帧间隔
        if(readPose(traj_type))
            frameGap = curPose.frameID - startID;
        
        // 读取剩余位姿
        while(readPose(traj_type))
            ;
        
        traj_file_.close();
        endID = curPose.frameID;
        
        std::cout << "\n========================= Details of trajectory ============================" << std::endl
                  << "\tformat: " << traj_format[traj_type] << std::endl
                  << "\tstartID: " << startID << std::endl
                  << "\tendID: " << endID << std::endl
                  << "\tframeGap: " << frameGap << std::endl
                  << "\tpose number: " << traj.size() << std::endl
                  << "==============================================================================" << std::endl;
    }

    void checkID(int b, int e)
    {
        if(b < startID || e > endID)
        {
            std::cout << "specified ID: (" << b << ", " << e << ") is beyond the range in trajectory!" << std::endl;
            exit(-1); 
        }
    }

    PoseNode getPose(int frameID)
    {
        assert(traj.count(frameID) > 0);
        return traj[frameID];
    }

    TrajType getTrajType() {return traj_type;}
    const int getFrameGap() { return frameGap; }
    const int getStartID() { return startID;}
    const int getEndID(){ return endID;}
    Eigen::Matrix4d getPoseMatrix(int frameID){ return getPose(frameID).tf;}

    void writeToFile(std::string out_file)
    {
        assert( fop.isExist(out_file) );
        
        std::ofstream out(out_file);
        if(!out.is_open() || out.fail())
            throw std::ios_base::failure("Error in opening file: " + out_file );

        writeHeader(out);

        for (int i = startID; i < traj.size(); i += frameGap)
            writePose(out, traj[i].frameID);
        
        traj_file_.close();
    }
    
    bool readPose(TrajType type)
    {
        switch(type)
        {
            case ROS_LOAM:
            {
                return readPoseLOAM();
            }
            case G2OT:
            {
                return readPoseG2OT();
            }
            case G2O:
            {
                return readPoseG2O();
            }
            case KITTI:
            {
                return readPoseKITTI();
            }
        }
    }

    //写入轨迹
    void writePose(std::ofstream &out, int frameID, std::string label = "ROS_LOAM_POSE")
    {
        double time = getPose(frameID).timestamp;
        out << label << ' ' << frameID << ' ';

        if(label == "VERTEX_SE3")
        {
            Eigen::Matrix4d tf = getPoseMatrix(frameID);
            Eigen::Matrix3d r = tf.block(0, 0, 3, 3);
            Eigen::Vector3d v = tf.block(0, 3, 3, 1);
            Eigen::Quaterniond q(r);
            out << v(0) << ' ' << v(1) << ' ' << v(2) << ' '
                << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << ' '
                << time;
        }
        else if(label == "ROS_LOAM_POSE")
        {
            Eigen::Matrix<double, 6, 1> poseVec = getPose(frameID).loamTrans;
            for(int i=0; i<6; ++i)
                out << poseVec[i];
            out << time;
        }
    }

    void writeHeader(std::ofstream &out, std::string label = "ROS_LOAM_POSE")
    {
        time_t now = time(NULL);
        out << "# DataName:\t" << data_name.c_str() << std::endl
            << "# Date:\t\t" << ctime(&now)
            << "# Data Format: {" << label;
        if(label == "VERTEX_SE3" || label == "iSAM_VERTEX")
            out << ", FrameID, Position[x|y|z], Quaternion[qx|qy|qz|qw],Timestamp(us)}" << std::endl;
        else if(label == "ROS_LOAM_POSE")
            out << ", FrameID, Euler[x|y|z], Position[x|y|z], Timestamp(us)}" << std::endl;
    }

private:
    std::map<int, PoseNode> traj;
    bool readPoseLOAM()
    {
        if(!(traj_file_ >> unused)) return false;
        
        traj_file_ >> curPose.frameID;
        for (int i = 0; i < 6; ++i)
            traj_file_ >> curPose.loamTrans[i];
        traj_file_ >> curPose.timestamp;

        curPose.tf = tt.euler2matrix(curPose.loamTrans);
        Eigen::Matrix3d r = curPose.tf.block(0, 0, 3, 3);
        Eigen::Quaterniond q(r);
        curPose.qua = q;
        curPose.pos = curPose.tf.block(0, 3, 3, 1);

        traj.insert( std::make_pair(curPose.frameID, curPose) );
        return true;
    }

    bool readPoseG2OT()
    {
        if(!(traj_file_ >> unused)) return false;

        Eigen::Quaterniond q;
        Eigen::Vector3d v;
        Eigen::Matrix3d r;

        // 9 number each line: {Frame Number, Position[x|y|z], Quaternion[qx|qy|qz|qw], Timestamp(us)}
        traj_file_ >> curPose.frameID >> v(0) >> v(1) >> v(2) >> q.x() >> q.y() >> q.z() >> q.w() >> curPose.timestamp;
        curPose.qua = q;
        curPose.pos = v;
        curPose.tf = Eigen::Matrix4d::Identity();
        r = q.toRotationMatrix();
        curPose.tf.block(0, 0, 3, 3) = r;
        curPose.tf.block(0, 3, 3, 1) = v;
    
        traj.insert( std::make_pair(curPose.frameID, curPose) );
        return true;
    }

    bool readPoseG2O()
    {
        if(!(traj_file_ >> unused)) return false;

        traj_file_ >> curPose.frameID;
        for(int i=0; i<7; ++i) traj_file_ >> tq(i);
        
        curPose.tf = tt.tq2matrix(tq);
        curPose.loamTrans = tt.matrix2euler(curPose.tf);
        //curPose.loamTrans = tt.tq2euler(tq);
        traj.insert( {curPose.frameID, curPose} );
        return true;
    }

    bool readPoseKITTI()
    {
        if(traj_file_.eof()) return false;
        
        std::string line;
        std::vector<std::string> st;

        getline(traj_file_, line);
        if(line.empty()) return false;

        boost::trim(line);
        boost::split(st, line, boost::is_any_of("\r\t "), boost::token_compress_on);

        if(st.size() != 12)
        {
            std::cout << "轨迹格式有问题" << std::endl;
            return false;
        }
        
        curPose.tf = Eigen::Matrix4d::Identity();
        
        for(int row=0; row<3; ++row)
            for(int col=0; col<4; ++col)
                curPose.tf(row, col) = (double)atof(st[row*4+col].c_str());
            
        curPose.frameID = (long long)pose_cnt++;
        traj.insert( std::make_pair(curPose.frameID, curPose) );
        
        // 这里检查一下位姿编号
        //std::cout << "frameID:" << curPose.frameID << std::endl;
        return true;
    }

private:
    TransformTool tt;
    int pose_cnt;
    std::string unused;
    long long startID;
    long long endID;
    int frameGap;
    PoseNode curPose;
    std::fstream traj_file_;
    std::string data_name;
    FileOperator fop;
    TrajType traj_type;
    int ignore_lines = 0;
    Vector7d tq;
};