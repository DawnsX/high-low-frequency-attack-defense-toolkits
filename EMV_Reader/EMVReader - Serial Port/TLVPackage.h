#pragma once

#include <list>
#include <vector>
#include <string>
#include <iostream>

#include <Windows.h>


// TLV�����
class TLVPackage
{
public:
    TLVPackage(bool isBuffer);
    //virtual ~TLVPackage();

    // ԭʹ�÷�����TLV����ʵ�庯���Ľӿڲ�
    static void Connector(const BYTE* pbRecv, DWORD dwRecv, std::list<TLVPackage *>& TLVList);

    //����TLVʵ��
    static void Construct(TLVPackage* WholePackage, std::list<TLVPackage *>& TLVList);

    // ���TLVʵ�������Ϣ
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
