import tkinter as tk
import unittest
from unittest.mock import Mock
from real_mapping import RealMappingWindow

class NavigationUITest(unittest.TestCase):
    def test_hover_buttons_and_board_status(self):
        root = tk.Tk(); root.withdraw()
        window = RealMappingWindow(root, 'unused.dll')
        try:
            root.update()
            window.send_navigation = Mock()
            window.show_navigation_menu(); root.update_idletasks()
            self.assertTrue(window.nav_menu.winfo_ismapped())
            self.assertTrue(window.nav_button.bind("<Enter>"))
            window.nav_start.invoke(); window.nav_end.invoke()
            labels = []
            def visit(widget):
                for child in widget.winfo_children():
                    if child.winfo_class() == 'TLabel':
                        labels.append(str(child.cget('text')))
                    if child.winfo_class() == 'TButton' and str(child.cget('text')) in ('Start（自动运行）', 'Stop（急停）', '手动控制'):
                        child.invoke()
                    visit(child)
            visit(window.win)
            self.assertIn('手柄 START 启停', labels)
            self.assertEqual([c.args[0] for c in window.send_navigation.call_args_list], [1, 2, 4, 6])
            window.queue.put(('navigation', 7, 42, 7, 2, 100, 12)); window.tick()
            self.assertIn('暂停，等待恢复', window.nav_status.get())
            self.assertIn('路径点 100', window.nav_status.get())
            window.send_navigation.reset_mock()
            device = window.device = Mock()
            window.disconnect()
            device.close.assert_called_once()
            window.send_navigation.assert_not_called()
        finally:
            window.nav_menu.withdraw(); root.destroy()
