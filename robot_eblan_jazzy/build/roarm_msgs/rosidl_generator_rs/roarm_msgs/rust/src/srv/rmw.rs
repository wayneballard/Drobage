#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "roarm_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__roarm_msgs__srv__MoveToXYZ_Request() -> *const std::ffi::c_void;
}

#[link(name = "roarm_msgs__rosidl_generator_c")]
extern "C" {
    fn roarm_msgs__srv__MoveToXYZ_Request__init(msg: *mut MoveToXYZ_Request) -> bool;
    fn roarm_msgs__srv__MoveToXYZ_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveToXYZ_Request>, size: usize) -> bool;
    fn roarm_msgs__srv__MoveToXYZ_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveToXYZ_Request>);
    fn roarm_msgs__srv__MoveToXYZ_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveToXYZ_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveToXYZ_Request>) -> bool;
}

// Corresponds to roarm_msgs__srv__MoveToXYZ_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveToXYZ_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub z: f64,

}



impl Default for MoveToXYZ_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !roarm_msgs__srv__MoveToXYZ_Request__init(&mut msg as *mut _) {
        panic!("Call to roarm_msgs__srv__MoveToXYZ_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveToXYZ_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { roarm_msgs__srv__MoveToXYZ_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { roarm_msgs__srv__MoveToXYZ_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { roarm_msgs__srv__MoveToXYZ_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveToXYZ_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveToXYZ_Request where Self: Sized {
  const TYPE_NAME: &'static str = "roarm_msgs/srv/MoveToXYZ_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__roarm_msgs__srv__MoveToXYZ_Request() }
  }
}


#[link(name = "roarm_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__roarm_msgs__srv__MoveToXYZ_Response() -> *const std::ffi::c_void;
}

#[link(name = "roarm_msgs__rosidl_generator_c")]
extern "C" {
    fn roarm_msgs__srv__MoveToXYZ_Response__init(msg: *mut MoveToXYZ_Response) -> bool;
    fn roarm_msgs__srv__MoveToXYZ_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveToXYZ_Response>, size: usize) -> bool;
    fn roarm_msgs__srv__MoveToXYZ_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveToXYZ_Response>);
    fn roarm_msgs__srv__MoveToXYZ_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveToXYZ_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveToXYZ_Response>) -> bool;
}

// Corresponds to roarm_msgs__srv__MoveToXYZ_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveToXYZ_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for MoveToXYZ_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !roarm_msgs__srv__MoveToXYZ_Response__init(&mut msg as *mut _) {
        panic!("Call to roarm_msgs__srv__MoveToXYZ_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveToXYZ_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { roarm_msgs__srv__MoveToXYZ_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { roarm_msgs__srv__MoveToXYZ_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { roarm_msgs__srv__MoveToXYZ_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveToXYZ_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveToXYZ_Response where Self: Sized {
  const TYPE_NAME: &'static str = "roarm_msgs/srv/MoveToXYZ_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__roarm_msgs__srv__MoveToXYZ_Response() }
  }
}






#[link(name = "roarm_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__roarm_msgs__srv__MoveToXYZ() -> *const std::ffi::c_void;
}

// Corresponds to roarm_msgs__srv__MoveToXYZ
#[allow(missing_docs, non_camel_case_types)]
pub struct MoveToXYZ;

impl rosidl_runtime_rs::Service for MoveToXYZ {
    type Request = MoveToXYZ_Request;
    type Response = MoveToXYZ_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__roarm_msgs__srv__MoveToXYZ() }
    }
}


