#pragma once
#include <cstdint>

class Communicator {
   public:

	void RecvUpdate(const uint8_t* buf);
    void Send();
    bool is_reply_ = false;

   private:
};
