abb_motion_program_exec_client_aio
==========================================================

ABB IRC5 Controller Motion Program AsyncIO command client.

This module contains types found on the ABB Robot Controller. Documentation is taken from the
ABB Robotics manual "Technical reference manual RAPID Instructions, Functions and Data types, 
Document ID: 3HAC 16581-1". Note that most of the following data structures are all direct copies of the 
underlying ABB types.

This module contains the ``MotionProgramExecClientAIO`` class. This class is functionally identical to 
``MotionProgramExecClient``, except it uses AsyncIO instead of synchronous blocking operations. This module
does not contain the various RAPID types. See :mod:`abb_motion_program_exec` for details on these classes. They
should be imported from :mod:`abb_motion_program_exec`.


abb_motion_program_exec.abb_motion_program_exec_client_aio
----------------------------------------------------------

.. automodule:: abb_motion_program_exec.abb_motion_program_exec_client_aio
    :members: