#pragma once

#include <list>
#include <vector>
#include <string>
#include <iostream>

#include <Windows.h>


// TLV打包类
class TLVPackage
{
public:
    TLVPackage(bool isBuffer);
    //virtual ~TLVPackage();

    // 原使用方法与TLV构造实体函数的接口层
    static void Connector(const BYTE* pbRecv, DWORD dwRecv, std::list<TLVPackage *>& TLVList);

    //构造TLV实体
    static void Construct(TLVPackage* WholePackage, std::list<TLVPackage *>& TLVList);

    // 输出TLV实体相关信息
    static void PrintTLVInfo(const TLVPackage* CurrentPackage);

public:
    std::vector<unsigned char> buffer;
    unsigned int bufferLength;

    std::vector<unsigned char> tag;
    unsigned int tagSize;

    std::vector<unsigned char> length;
    unsigned int lengthSize;

    std::vector<unsigned char> value;
    unsigned int valueSize;

    bool BufferOnly;
};
