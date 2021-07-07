
#include <memory>

#include "../core/core.hpp"
#include "../interface/interface.hpp"
#include "../plugins/navigation/navigation.hpp"

int main(int argc, char const *argv[])
{
    std::shared_ptr<simulator::core::Core> core_ptr = std::make_shared<simulator::core::Core>();
    std::shared_ptr<simulator::plugin::Navigation> nav_ptr = std::make_shared<simulator::plugin::Navigation>(core_ptr);
    return 0;
}
