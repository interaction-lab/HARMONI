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
- ActionClient
```
from action_client import ActionClient

action_client = ActionClient()
```

- ActionServer
```
from action_server import ActionServer

server = ActionServer()
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
