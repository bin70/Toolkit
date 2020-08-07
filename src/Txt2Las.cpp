#include <utils/argparse.hpp>
#include <io/FileOperator.hpp>
#include <io/PCDOperator.hpp>
#include <io/LasOperator.hpp>
#include <point_cloud/common.h>

using namespace std;
FileOperator fop;
ArgumentParser parser;

int main(int argc, const char **argv)
{
    parser.addArgument("-i", "--input_dir", true);
    parser.addArgument("-o", "--output_dir");
    parser.parse(argc, argv);

    string input = parser.get("input_dir");
    vector<string> files = fop.openDir( input );
    PointCloud::Ptr cloud(new PointCloud);
    PointType p;
    for(int i=0; i<files.size(); ++i)
    {
        ifstream file(files[i].c_str());
        cout << "merging " << files[i] << endl;
        
        while(file >> p.x)
        {
            file >> p.y >> p.z >> p.intensity;
            cloud->points.push_back(p);
        }    
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    string out_dir = input;
    if(parser.count("output_dir"))
        out_dir = parser.get("output_dir");
    
    saveLasFile(out_dir+"/Merged.las", cloud);
    return 0;
}