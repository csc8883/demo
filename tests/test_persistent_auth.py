import time

from fastapi import FastAPI, Depends
from fastapi.testclient import TestClient

from backend.core import security, dependencies
from backend.routers import auth


def test_cookie_survives_manager_restart_and_logout_revokes_it(tmp_path, monkeypatch):
    monkeypatch.setattr(security, 'SYSTEM_FILE', tmp_path / 'users.json')
    monkeypatch.setattr(security, 'USER_DATA_DIR', tmp_path / 'data')
    manager = security.UserManager()
    monkeypatch.setattr(auth, 'user_manager', manager)
    monkeypatch.setattr(auth, 'authenticate_user', manager.authenticate)
    monkeypatch.setattr(auth, 'build_login_response', lambda name, record: {
        'status': 'success', 'data': {'name': name, 'token': manager.create_session(name)}})
    app = FastAPI()
    app.include_router(auth.router)
    client = TestClient(app)
    assert client.get('/api/auth/me').status_code == 401
    assert client.post('/api/auth/login', data={'username': 'admin', 'password': 'wrong'}).json()['status'] == 'error'
    response = client.post('/api/auth/login', data={'username': 'admin', 'password': 'admin'})
    assert 'HttpOnly' in response.headers['set-cookie']
    token = response.json()['data']['token']
    restarted = security.UserManager()
    monkeypatch.setattr(auth, 'user_manager', restarted)
    monkeypatch.setattr(dependencies, 'user_manager', restarted)
    @app.get('/protected')
    def protected(user=Depends(dependencies.get_current_user)):
        return {'name': user}
    assert client.get('/api/auth/me').json()['data']['name'] == 'admin'
    assert client.get('/protected').json()['name'] == 'admin'
    assert restarted.validate_session('forged') is None
    client.post('/api/auth/logout')
    assert restarted.validate_session(token) is None
    assert client.get('/protected').status_code == 401
    token = restarted.create_session('admin')
    with restarted._session_db() as db:
        db.execute('UPDATE sessions SET expires=?', (time.time() - 1,))
    assert restarted.validate_session(token) is None
