from __future__ import print_function
from __future__ import division
from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QCoreApplication, Qt
from easy_handeye.handeye_client import HandeyeClient
try:
    from python_qt_binding.QtGui import QWidget, QApplication, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel, QPushButton
except ImportError:
    from python_qt_binding.QtWidgets import QWidget, QApplication, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel, QPushButton
import rospy
import sys


class AutoSampleMoveGUI(QWidget):
    NOT_INITED_YET = 0
    INIT_OK = 1
    INIT_BAD = 2
    TRYING_PLAN = 3
    PLAN_OK_EXECUTING = 4
    EXEC_OK = 5
    EXEC_FAIL = 6
    NO_VALID_PLANS = 7

    def __init__(self):
        super(AutoSampleMoveGUI, self).__init__()
        self.handeye_client = HandeyeClient()
        self.current_target_pose = -1  # -1 means "home" per original
        self.target_poses = None
        self.state = AutoSampleMoveGUI.NOT_INITED_YET

        # UI
        self.layout = QVBoxLayout()
        self.top_layout = QHBoxLayout()
        self.btns_layout = QHBoxLayout()

        self.progress_bar = QProgressBar()
        self.pose_number_lbl = QLabel('0/0')
        self.status_lbl = QLabel('Welcome')
        self.status_lbl.setAlignment(Qt.AlignCenter)
        self.status_lbl.setWordWrap(True)

        self.init_btn = QPushButton('Initialize')
        self.init_btn.clicked.connect(self.handle_init)

        self.sample_btn = QPushButton('Sample movement')
        self.sample_btn.clicked.connect(self.handle_sample_and_execute)
        self.sample_btn.setEnabled(False)

        self.skip_btn = QPushButton('Skip')
        self.skip_btn.clicked.connect(self.handle_skip)
        self.skip_btn.setEnabled(False)

        self.top_layout.addWidget(self.pose_number_lbl)
        self.btns_layout.addWidget(self.init_btn)
        self.btns_layout.addWidget(self.sample_btn)
        self.btns_layout.addWidget(self.skip_btn)

        self.layout.addWidget(self.progress_bar)
        self.layout.addLayout(self.top_layout)
        self.layout.addWidget(self.status_lbl)
        self.layout.addLayout(self.btns_layout)

        self.setLayout(self.layout)
        self.setWindowTitle('Auto Sample & Move')
        self.update_ui()
        self.show()

    # ---------- UI helpers ----------
    def update_progress(self):
        count = len(self.target_poses) if self.target_poses else 1
        self.progress_bar.setMaximum(count)
        # show +1 because current_target_pose is index; if -1 (home), show 0
        shown_idx = max(self.current_target_pose, -1) + 1
        self.progress_bar.setValue(min(shown_idx, count))
        self.pose_number_lbl.setText('{}/{}'.format(shown_idx, count))

    def set_status(self, text):
        self.status_lbl.setText(text)
        QCoreApplication.processEvents()

    def update_ui(self):
        self.update_progress()
        # Buttons availability
        self.init_btn.setEnabled(self.state in (self.NOT_INITED_YET, self.INIT_BAD))
        can_sample = self.state in (self.INIT_OK, self.EXEC_OK, self.EXEC_FAIL)
        self.sample_btn.setEnabled(can_sample)
        self.skip_btn.setEnabled(can_sample)

        # Status text
        if self.state == self.NOT_INITED_YET:
            self.set_status('Click "Initialize" to detect poses from the current robot position.')
        elif self.state == self.INIT_OK:
            self.set_status('Initialized. Click "Sample movement" to plan and (if possible) move automatically.')
        elif self.state == self.INIT_BAD:
            self.set_status('Cannot calibrate from current position. Reposition the robot and click "Initialize" again.')
        elif self.state == self.TRYING_PLAN:
            self.set_status('Planning to next pose…')
        elif self.state == self.PLAN_OK_EXECUTING:
            self.set_status('Plan looks good. Executing…')
        elif self.state == self.EXEC_OK:
            self.set_status('Pose reached. You can sample data and then continue.')
        elif self.state == self.EXEC_FAIL:
            self.set_status('Execution failed. Try "Sample movement" again or "Skip".')
        elif self.state == self.NO_VALID_PLANS:
            self.set_status('No valid plans for any remaining poses.')

    # ---------- Behavior ----------
    def handle_init(self):
        self.state = self.TRYING_PLAN  # temporary to show activity
        self.update_ui()
        res = self.handeye_client.check_starting_pose()
        if res.can_calibrate:
            self.state = self.INIT_OK
        else:
            self.state = self.INIT_BAD
        self.current_target_pose = res.target_poses.current_target_pose_index
        self.target_poses = res.target_poses.target_poses
        self.update_ui()

    def handle_skip(self):
        if self.target_poses is None:
            return
        next_idx = self.current_target_pose + 1
        res = self.handeye_client.select_target_pose(next_idx)
        self.current_target_pose = res.target_poses.current_target_pose_index
        self.target_poses = res.target_poses.target_poses
        # stay in INIT_OK-like state so user can try again
        if self.state not in (self.INIT_OK, self.EXEC_OK, self.EXEC_FAIL):
            self.state = self.INIT_OK
        self.update_ui()

    def handle_sample_and_execute(self):
        """
        Single action: choose current (or next) target pose, try plan; if plan fails,
        automatically advance through remaining poses until one plans; then execute automatically.
        """
        if self.state not in (self.INIT_OK, self.EXEC_OK, self.EXEC_FAIL):
            # Ensure we have a pose list
            self.handle_init()
            if self.state != self.INIT_OK:
                return

        if not self.target_poses:
            self.state = self.NO_VALID_PLANS
            self.update_ui()
            return

        total = len(self.target_poses)
        attempts = 0
        planned = False

        while attempts < total:
            self.state = self.TRYING_PLAN
            self.update_ui()

            # Always make sure current index is selected on the backend
            sel = self.handeye_client.select_target_pose(self.current_target_pose)
            self.current_target_pose = sel.target_poses.current_target_pose_index
            self.target_poses = sel.target_poses.target_poses
            self.update_progress()

            plan_res = self.handeye_client.plan_to_selected_target_pose()
            if plan_res.success:
                planned = True
                break
            else:
                # Advance to the next pose and try again
                next_idx = self.current_target_pose + 1
                sel = self.handeye_client.select_target_pose(next_idx)
                self.current_target_pose = sel.target_poses.current_target_pose_index
                self.target_poses = sel.target_poses.target_poses
                attempts += 1

        if not planned:
            self.state = self.NO_VALID_PLANS
            self.update_ui()
            return

        # Execute automatically
        self.state = self.PLAN_OK_EXECUTING
        self.update_ui()
        exec_res = self.handeye_client.execute_plan()
        if exec_res.success:
            self.state = self.EXEC_OK
        else:
            self.state = self.EXEC_FAIL
        self.update_ui()


class RqtAutoSampleMove(Plugin):
    def __init__(self, context):
        super(RqtAutoSampleMove, self).__init__(context)
        self.setObjectName('AutoSampleMove')

        rospy.sleep(1.0)

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        self._widget = AutoSampleMoveGUI()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass


if __name__ == '__main__':
    NODE_NAME = 'easy_handeye_auto_mover'

    rospy.init_node(NODE_NAME)
    while rospy.get_time() == 0.0:
        pass

    qapp = QApplication(sys.argv)
    gui = AutoSampleMoveGUI()
    gui.show()
    sys.exit(qapp.exec_())
