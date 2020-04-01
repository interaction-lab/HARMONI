# Class skeleton

- Import libraries
- Class attributes
- Class operations sorted:
    1. Init
    2. Setup
    3. Callback
    4. Check
    5. Get
    6. Send
    7. Handle

# Usage of class
List of classes:
```
- Controller
```
from controller import Controller

controller = Controller()
```
- ExternalService
```
from child import ExternalService

ext_service = ExternalService()
```
- InternalService
```
from child import InternalService

int_service = InternalService()
```
- HardwareReading
```
from child import HardwareReading

hw_read = HardwareReading()
```
- HardwareControl
```
from child import HardwareControl

hw_control = HardwareControl()
```
Subclasses:
@name: the name of the hardware element, will be the name of the server
@service_manager: service managers should have the following fuctionality:
            service_manager.test() # sends default or example action
            service_manager.do(data) # processes data and does action
            service_manager.reset_init() # Resets hardware variables to initial state

            service_manager.action_completed# True if action completed
            service_manager.cont = Bool # Used IF logic can dictate control flow
            service_manager.result_msg = String # 
