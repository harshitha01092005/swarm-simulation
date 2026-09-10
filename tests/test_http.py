from concurrent.futures import Future
from http.client import HTTPConnection
import json
from pathlib import Path
import tempfile
import unittest
from drone_swarm.gui.server import SwarmHTTPServer


class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix='swarm-http-test-')
        Path(cls.directory.name, 'index.html').write_text('<html>Swarm</html>')
        def submit(command):
            result = Future()
            if command['action'] == 'bad':
                result.set_exception(ValueError('Unknown action'))
            else:
                result.set_result({'accepted': True, 'action': command['action']})
            return result
        cls.server = SwarmHTTPServer(('127.0.0.1', 0), lambda: {'connected': True}, submit,
                                     web_root=cls.directory.name)
        cls.thread = cls.server.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)
        cls.directory.cleanup()

    def request(self, method, path, body=None, headers=None):
        client = HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        client.request(method, path, body, headers or {})
        response = client.getresponse()
        status, payload = response.status, response.read()
        client.close()
        return status, payload

    def test_state_and_local_static_page(self):
        status, data = self.request('GET', '/api/state')
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(data)['connected'])
        status, data = self.request('GET', '/')
        self.assertEqual(status, 200)
        self.assertIn(b'Swarm', data)

    def test_command_ack_and_validation_failure(self):
        for action, expected in [('start', 200), ('bad', 400)]:
            status, data = self.request('POST', '/api/command', json.dumps({'action':action}),
                                        {'Content-Type':'application/json'})
            self.assertEqual(status, expected)
            self.assertIn('action' if expected == 200 else 'error', json.loads(data))

    def test_foreign_origin_host_and_path_traversal_rejected(self):
        self.assertEqual(self.request('GET', '/api/state', headers={'Host':'attacker.example'})[0], 403)
        self.assertEqual(self.request('POST', '/api/command', '{}',
                         {'Content-Type':'application/json','Origin':'https://attacker.example'})[0], 403)
        self.assertEqual(self.request('GET', '/../index.html')[0], 404)

    def test_bad_json_type_content_type_and_oversized_body(self):
        self.assertEqual(self.request('POST','/api/command','[]',{'Content-Type':'application/json'})[0],400)
        self.assertEqual(self.request('POST','/api/command','{',{'Content-Type':'application/json'})[0],400)
        self.assertEqual(self.request('POST','/api/command','{}',{'Content-Type':'text/plain'})[0],400)
        self.assertEqual(self.request('POST','/api/command',None,
                         {'Content-Type':'application/json','Content-Length':str(13*1024*1024)})[0],413)
