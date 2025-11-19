#include "gui-manager-base.hpp"
#include "gui-manager.hpp"

unique_ptr<GUI_ManagerBase> GUI_ManagerBase::create() {
    return make_unique<GUI_Manager>();
}