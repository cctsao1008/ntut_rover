/**
 ******************************************************************************
 * @file      socket.c
 * @brief     C Source file of socket.c.
 * @details   This file including all API functions's 
 *            implement of socket.c.	
 *
 * @copyright
 ******************************************************************************
 */
 
/*-----------------------------------------------------------------------------
 Section: Includes
 ----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

/**
 ******************************************************************************
 * @brief
 * @param[in]  *pHostName   :
 * @param[in]  port         :
 * @param[in]  time_out     :
 *
 * @retval     0:
 * @retval    >0:
 ******************************************************************************
 */
unsigned int
socket_init(const char *pHostName,
        unsigned short port,
        int time_out)
{

    int sockfd = -1;
    struct sockaddr_in server_addr;
    struct hostent *host;

    time_out = (time_out == 0) ? 1000 * 15 : time_out;

    do
    {
        if((host = gethostbyname(pHostName)) == NULL)
        {
            printf("%s gethostname error\n", __FUNCTION__);
            break;
        }

        if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
        {
            printf("socket error:%s\a\n", strerror(errno));
            break;
        }

        memset(&server_addr, 0x00, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr = *((struct in_addr*)host->h_addr);

        if (connect(sockfd, (struct sockaddr *)(&server_addr),
                sizeof(struct sockaddr)) == -1)
        {
            printf("connect error:%s\n", strerror(errno));
            break;
        }
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&time_out, sizeof(int));
        return (unsigned int)sockfd;
    } while(0);

    return 0u;
}

/**
 ******************************************************************************
 * @brief
 * @param[in]  socket   :
 * @param[in]  *pbuf    :
 * @param[in]  size     :
 *
 * @retval     -1   :
 * @retval     size :
 ******************************************************************************
 */
int
socket_send(unsigned int socket,
        const char *pbuf,
        int size)
{
    if (socket == 0)
    {
        return -1;
    }
    return send(socket, pbuf, size, 0);
}

/**
 ******************************************************************************
 * @brief
 * @param[in]  socket   :
 * @param[in]  *pbuf    :
 * @param[in]  size     :
 *
 * @retval     -1   :
 * @retval     size :
 ******************************************************************************
 */
int
socket_recv(unsigned int socket,
        char *pbuf,
        int size)
{
    if (socket == 0)
    {
        return -1;
    }
    return recv(socket, pbuf, size, 0);
}

/**
 ******************************************************************************
 * @brief      Ì×½Ó×Ö¹Ø±Õ
 * @param[in]  socket   : Ì×½Ó×Ö¾ä±ú
 *
 * @return  None
 ******************************************************************************
 */
void
socket_close(unsigned int socket)
{
    if (socket != 0)
    {
        close(socket);
    }
}

/*--------------------------------socket.c-----------------------------------*/
