#pragma once


#include "MsgStrctAgv.h"
#include "MsgStrctCC.h"
#include "global.h"


#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <exception>
#include <functional>
#include <glog/logging.h>
#include <memory>
#include <mutex>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <netinet/tcp.h>
#include <utility>
#include <vector>

class Client {
private:
  int m_fd;
  int m_agv_id;
  std::thread m_t;
  bool isconnect = false;

public:
  Client(){}
  
  Client(int fd, int agv){
    m_fd = fd;
    m_agv_id = agv;
    isconnect = true;
    m_t = std::thread([=](){func();});
  }

  Client(Client &&in) {
    this->m_agv_id = in.m_agv_id;
    this->m_fd = in.m_fd;
    this->m_t = std::move(in.m_t);
    this->isconnect = true;
  }

  Client& operator = (Client&& in) {
    this->m_agv_id = in.m_agv_id;
    this->m_fd = in.m_fd;
    this->m_t = std::move(in.m_t);
    this->isconnect = true;
    return *this;
  }

  void send(char *data, int length) {
    ::send(m_fd, data, length, 0);
  }

  bool connected(){
    return isconnect;
  }

  ~Client() {
    close(m_fd);
    isconnect = false;
    if(m_t.joinable()) m_t.join();
  }
private: 
  void sendHeartBeat(){
    MSG_CC::HeartBeat heartbeat;
    send((char *)&heartbeat, sizeof(heartbeat));
  }

  void func() {
    // 接收消息
		char buf[1024];
    static auto last_time = std::chrono::steady_clock::now();
    while (isconnect) {
      // send heartbeat
      auto curr_time = std::chrono::steady_clock::now();
      std::chrono::duration<double> e = curr_time - last_time;
      if(e.count() > 1) {
        sendHeartBeat();
        last_time = curr_time;
      }

			memset(buf, 0, sizeof(buf));
			int length = recv(m_fd, buf, 1024, 0);
      if(length == 0){
        LOG(INFO) << "disconnected " << this;
        isconnect = false;
        // 说明这个client断开连接了, 删除
        Global::remove(m_agv_id);
        break;
      }
			// TODO：解析消息
			MSG_AGV::BaseData *base_agv = (MSG_AGV::BaseData *) buf;
      // LOG(INFO) << "recv msg of "<< m_agv_id << ", type " << base_agv->m_head;
			switch (base_agv->m_head) {
				// 位置和速度
        case MSG_AGV::_agvStatusHeadEnum: {
					MSG_AGV::AgvStatus *agv = (MSG_AGV::AgvStatus *) buf;
					Global::set_agv_status(m_agv_id, AGVstatus(agv->m_x, agv->m_y, agv->m_theta, agv->m_v, agv->m_w));
          // LOG(INFO) << m_agv_id << " pos, " << agv->m_x << ", " << agv->m_y << ", " << agv->m_theta;
          break;
        }
        // agv到达
        case MSG_AGV::_agvArriveHeadEnum: {
          MSG_AGV::AgvArrive *agv = (MSG_AGV::AgvArrive *) buf;
  				Global::set_agv_job_state(m_agv_id, JOBSTATE::finish);
          LOG(INFO) << m_agv_id << " arrive";
          break;
        }
        // agv接受任务
        case MSG_AGV::_agvTaskResultHeadEnum: {
          MSG_AGV::AgvTaskResult *agv = (MSG_AGV::AgvTaskResult *) buf;
          if(agv->m_result == MSG_AGV::AgvTaskResult::TaskResultSuccess) {
            Global::set_agv_job_state(m_agv_id, JOBSTATE::working);
            LOG(INFO) << m_agv_id << " receive task";  
          } else {
            LOG(INFO) << m_agv_id << " refuse task: " << (int)agv->m_result;  
          }
          break;
        }
        // agv改变模式成功
        case MSG_AGV::_agvSetModeResultHeadEnum: {
          MSG_AGV::AgvSetModeResult *res = (MSG_AGV::AgvSetModeResult *) buf;
          if(res->m_result == MSG_AGV::AgvSetModeResult::SuccessResultType) {
            LOG(INFO) << m_agv_id << " set mode succeful"; 
          } else {
            LOG(INFO) << m_agv_id << " set mode failed"; 
          }
          break;
        }
      } 
    }
  }
};

class TcpServer {
public:
  TcpServer(int port) {
    listen_sock = socket(PF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
		int flag = 1;
		setsockopt(listen_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));

    serv_addr.sin_family = PF_INET;
    inet_pton(PF_INET, "192.168.1.227", &(serv_addr.sin_addr));
    serv_addr.sin_port = htons(port);
    int res =
        bind(listen_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    if (res != 0) {
      LOG(ERROR) << "error binding socket" << strerror(errno);
      exit(1);
    }
    listen(listen_sock, 10);
		LOG(INFO) << "open server listening on " << port;
  }

  void start() {
    t_listen = std::thread([=]() {
      while (1) {
        struct sockaddr_in clientAddress;
        socklen_t clientAddrLen = sizeof(clientAddress);
        int clientfd = accept(
            listen_sock, (struct sockaddr *)&clientAddress, &clientAddrLen);
				int flag = 1;
				setsockopt(clientfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
        char clientIP[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &clientAddress.sin_addr, clientIP, sizeof(clientIP));
        std::string s = clientIP;
        std::size_t pos = s.find_last_of('.');
        std::string lastPart = s.substr(pos + 1);
        int agvid = std::stoi(lastPart) %100;
        LOG(INFO) << "get connected requeset from " << agvid << ":"
                  << ntohs(clientAddress.sin_port);
        
        std::unique_lock<std::mutex> lock(mutex_);
        auto iter = clients_.find(agvid);

        if(iter != clients_.end() && iter->second->connected()) {
          close(clientfd);
          LOG(INFO) << "already, reject connect. " << iter->second->connected() << " agv: " << static_cast<void*>(iter->second.get());
        } else {
          auto ptr = std::make_shared<Client>(clientfd, agvid);
          clients_.insert(std::make_pair(agvid, ptr));
          LOG(INFO) << "save client, fd: " << clientfd << " agv: " << static_cast<void*>(ptr.get());
        }
      }
    });
  }



	// 根据车号发送消息
	void send(int agvid, char *data, int length) {
    auto iter = clients_.find(agvid);
		if(iter != clients_.end()) {
      iter->second->send(data, length);
    }
	}

  ~TcpServer() { 
    close(listen_sock);
    if(t_listen.joinable()) t_listen.join(); 
  }

private:
  int listen_sock;
  struct sockaddr_in serv_addr;
  
  std::unordered_map<int, std::shared_ptr<Client>> clients_; // 车号对应的fd和线程
  std::mutex mutex_;     // clients列表的锁

	std::thread t_listen;
};
