//
// Created by cyz on 18-11-16.
//
#include "netinet/in.h"
#include <arpa/inet.h>

#include "malloc.h"
#include "pcapReader.h"
#include <stdio.h>

using namespace std;

void pcapReader::prinfPcapFileHeader(pcapReader::pcap_file_header *pfh)
{
    if (pfh==NULL) { return; }
    printf("=====================\n"
           "magic:0x%0x\n"
           "version_major:%u\n"
           "version_minor:%u\n"
           "thiszone:%d\n"
           "sigfigs:%u\n"
           "snaplen:%u\n"
           "linktype:%u\n"
           "=====================\n",
           pfh->magic, pfh->version_major, pfh->version_minor,
           pfh->thiszone, pfh->sigfigs, pfh->snaplen, pfh->linktype);

}

void pcapReader::printfPcapHeader(pcapReader::pcap_header *ph)
{
    if (ph==NULL) { return; }
    printf("=====================\n"
           "ts.timestamp_s:%u\n"
           "ts.timestamp_ms:%u\n"
           "capture_len:%u\n"
           "len:%d\n"
           "=====================\n",
           ph->ts.timestamp_s, ph->ts.timestamp_ms, ph->capture_len, ph->len);

}

void pcapReader::printPcap(void *data, size_t size)
{
    unsigned short iPos = 0;
    //int * p = (int *)data;
    //unsigned short* p = (unsigned short *)data;
    if (data==NULL) { return; }

    printf("\n===data:0x%x,len:%lu=========",data,size);
    for (iPos=0; iPos < size/sizeof(unsigned short); iPos++)
    {
        //printf(" %x ",(int)( * (p+iPos) ));
        // unsigned short a = ntohs(p[iPos]);
        unsigned short a = ntohs( *((unsigned short *)data + iPos ) );

        if (iPos%8==0) printf("\n");
        if (iPos%4==0) printf(" ");
        printf("-%d-",a);
//        printf("%04x",a);
    }


//    for (iPos=0; iPos <= size/sizeof(int); iPos++)
//    {
//        //printf(" %x ",(int)( * (p+iPos) ));
//        int a = ntohl(p[iPos]);
//        //int a = ntohl( *((int *)data + iPos ) );
//        if (iPos %4==0) printf("\n");
//        printf("%08x ",a);
//    }


    printf("\n============\n");
}


#define MAX_ETH_FRAME 1514
#define ERROR_FILE_OPEN_FAILED -1
#define ERROR_MEM_ALLOC_FAILED -2
#define ERROR_PCAP_PARSE_FAILED -3

int pcapReader::readpcapfile(char *filepath)
{
    printf("sizeof:int %lu,unsigned int %lu,char %lu,unsigned char %lu,short:%lu,unsigned short:%lu\n",
           sizeof(int),
           sizeof(unsigned int),
           sizeof(char),
           sizeof(unsigned char),
           sizeof(short),
           sizeof(unsigned short));

    pcap_file_header pfh;
    pcap_header ph;
    int count=0;
    void * buff = NULL;
    int readSize=0;
    int ret = 0;

    FILE *fp = fopen(filepath, "rb");
    if (fp==NULL)
    {
        fprintf(stderr, "Open file %s error.",filepath);
        ret = ERROR_FILE_OPEN_FAILED;
        goto ERROR;
    }
    fread(&pfh, sizeof(pcap_file_header), 1, fp);
    prinfPcapFileHeader(&pfh);
    //fseek(fp, 0, sizeof(pcap_file_header));

    buff = (void *)malloc(MAX_ETH_FRAME);
    for (count=1; ; count++)
    {
        memset(buff,0,MAX_ETH_FRAME);

        //read pcap header to get a packet
        // get only a pcap head count .
        readSize=fread(&ph, sizeof(pcap_header), 1, fp);
        if (readSize<=0) { break; }
        printfPcapHeader(&ph);

        if (buff==NULL)
        {
            fprintf(stderr, "malloc memory failed.\n");
            ret = ERROR_MEM_ALLOC_FAILED;
            goto ERROR;
        }

        //get a packet contents.
        //read ph.capture_len bytes.
        readSize=fread(buff, 1, ph.capture_len, fp);
        if (readSize != ph.capture_len)
        {
            free(buff);
            fprintf(stderr, "pcap file parse error.\n");
            ret = ERROR_PCAP_PARSE_FAILED;
            goto ERROR;
        }
        printPcap(buff, ph.capture_len);

        printf("===count:%d,readSize:%d===\n",count,readSize);

        if (feof(fp) || readSize <=0 ) { break; }
    }

    ERROR:
    //free
    if (buff)
    {
        free(buff);
        buff=NULL;
    }
    if (fp)
    {
        fclose(fp);
        fp=NULL;
    }

    return ret;
}