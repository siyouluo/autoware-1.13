#ifndef _MKPATH_H
#define _MKPATH_H
//https://blog.csdn.net/rathome/article/details/78870694
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <errno.h>

namespace light
{
int mkpath(std::string s,mode_t mode=0755)
{
    size_t pre=0,pos;
    std::string dir;
    int mdret;

    if(s[s.size()-1]!='/'){
        // force trailing / so we can handle everything in loop
        s+='/';
    }

    while((pos=s.find_first_of('/',pre))!=std::string::npos){
        dir=s.substr(0,pos++);
        pre=pos;
        if(dir.size()==0) continue; // if leading / first time is 0 length
        if((mdret=::mkdir(dir.c_str(),mode)) && errno!=EEXIST){
            return mdret;
        }
    }
    return mdret;
}

/*
 * 注：使用时，light::mkdir, 易与::mkdir 混淆，固去掉了
int mkdir(std::string s,mode_t mode)
{
    light::mkpath(s, mode);
}
*/
bool sysmkdir(const char* path, mode_t mode=S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO)
{   
    int isCreate = ::mkdir(path,mode);
    if( !isCreate )
        printf("create path:%s\n",path);
    else
        printf("create path:%s failed! error code : %x \n",path,errno);

}
}
#endif // _MKPATH_H
