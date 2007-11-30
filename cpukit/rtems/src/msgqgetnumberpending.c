/*
 *  Message Queue Manager
 *
 *
 *  COPYRIGHT (c) 1989-2007.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/system.h>
#include <rtems/score/sysstate.h>
#include <rtems/score/chain.h>
#include <rtems/score/isr.h>
#include <rtems/score/coremsg.h>
#include <rtems/score/object.h>
#include <rtems/score/states.h>
#include <rtems/score/thread.h>
#include <rtems/score/wkspace.h>
#if defined(RTEMS_MULTIPROCESSING)
#include <rtems/score/mpci.h>
#endif
#include <rtems/rtems/status.h>
#include <rtems/rtems/attr.h>
#include <rtems/rtems/message.h>
#include <rtems/rtems/options.h>
#include <rtems/rtems/support.h>

/*PAGE
 *
 *  rtems_message_queue_get_number_pending
 *
 *  This directive returns the number of messages pending.
 *
 *  Input parameters:
 *    id    - queue id
 *    count - return area for count
 *
 *  Output parameters:
 *    count             - number of messages removed ( 0 = empty queue )
 *    RTEMS_SUCCESSFUL - if successful
 *    error code        - if unsuccessful
 */

rtems_status_code rtems_message_queue_get_number_pending(
  Objects_Id  id,
  uint32_t   *count
)
{
  register Message_queue_Control *the_message_queue;
  Objects_Locations               location;

  if ( !count )
    return RTEMS_INVALID_ADDRESS;

  the_message_queue = _Message_queue_Get( id, &location );
  switch ( location ) {

    case OBJECTS_LOCAL:
      *count = the_message_queue->message_queue.number_of_pending_messages;
      _Thread_Enable_dispatch();
      return RTEMS_SUCCESSFUL;

#if defined(RTEMS_MULTIPROCESSING)
    case OBJECTS_REMOTE:
      _Thread_Executing->Wait.return_argument = count;

      return _Message_queue_MP_Send_request_packet(
          MESSAGE_QUEUE_MP_GET_NUMBER_PENDING_REQUEST,
          id,
          0,                               /* buffer not used */
          0,                               /* size */
          0,                               /* option_set not used */
          MPCI_DEFAULT_TIMEOUT
        );
#endif

    case OBJECTS_ERROR:
      break;
  }

  return RTEMS_INVALID_ID;
}
