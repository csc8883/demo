"""One-time local account consolidation; password is read from ADMIN_PASSWORD.

Run with both application backends stopped and Neo4j/MySQL running.
Backups contain private data and MUST stay outside either Git repository.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
from datetime import datetime


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--assistant', type=Path, default=Path('D:/ai_tools/llm-graph-builder'))
    parser.add_argument('--backup', type=Path, required=True)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    password = os.environ.get('ADMIN_PASSWORD', '')
    if len(password) < 10:
        raise SystemExit('Set ADMIN_PASSWORD (at least 10 characters)')
    backup = args.backup.resolve()
    for project in (root, args.assistant.resolve()):
        if backup.is_relative_to(project):
            raise SystemExit('Backup must be outside both projects')
    backup.mkdir(parents=True, exist_ok=False)
    backend = args.assistant / 'backend'
    sys.path.insert(0, str(backend))
    from dotenv import load_dotenv
    load_dotenv(backend / '.env', override=True)
    from sqlalchemy import inspect, text, select
    from src.db.session import create_database_engine
    from src.auth.service import hash_password
    from src.workspace.graph import KnowledgeStore
    engine = create_database_engine()
    inspector = inspect(engine)
    tables = inspector.get_table_names()
    with engine.connect() as conn:
        snapshot = {table: [dict(row) for row in conn.execute(text(f'SELECT * FROM `{table}`')).mappings()] for table in tables}
    (backup / 'mysql-before.json').write_text(json.dumps(snapshot, default=str, ensure_ascii=False, indent=2), 'utf-8')
    store = KnowledgeStore()
    graph = store.query('MATCH (n) WHERE n.owner_id IS NOT NULL OR n.owner IS NOT NULL RETURN elementId(n) AS id, properties(n) AS properties')
    (backup / 'neo4j-ownership-before.json').write_text(json.dumps(graph, default=str, ensure_ascii=False, indent=2), 'utf-8')
    users_file = root / 'users.json'
    shutil.copy2(users_file, backup / 'users-before.json')
    data = root / 'userdata'
    users = json.loads(users_file.read_text('utf-8'))
    copies = []
    targets = {}
    sources = [p for p in data.iterdir() if p.is_dir() and p.name not in {'admin', 'pointcloud_lod'}]
    for folder in sources:
        assert folder.resolve().parent == data.resolve()
        for source in folder.rglob('*'):
            if not source.is_file():
                continue
            relative = source.relative_to(folder)
            target = data / 'admin' / relative
            previous = targets.get(relative, target if target.exists() else None)
            if previous and digest(previous) != digest(source):
                raise RuntimeError(f'Conflicting file requires explicit merge: {relative}')
            targets[relative] = source
            copies.append((source, target))
    # Preflight owner-scoped unique keys before touching either database.
    for table in tables:
        for constraint in inspector.get_unique_constraints(table):
            columns = constraint['column_names']
            if 'owner_id' not in columns:
                continue
            seen = set()
            for row in snapshot[table]:
                key = tuple(row[col] for col in columns if col != 'owner_id')
                if any(value is None for value in key):
                    continue
                if key in seen:
                    raise RuntimeError(f'Owner merge conflicts in {table}: {columns}')
                seen.add(key)
    summary = {'planning_files': len(copies), 'mysql_rows': {table: len(rows) for table, rows in snapshot.items()}, 'graph_nodes': len(graph)}
    (backup / 'inventory.json').write_text(json.dumps(summary, indent=2), 'utf-8')
    print(json.dumps(summary))
    if not args.apply:
        store.close()
        return
    # Preserve IDs and provenance, reassign only resource ownership. Old user rows
    # remain disabled so audit history and foreign keys remain valid.
    from sqlalchemy.orm import Session
    from src.models.identity import User, Role, UserRole
    from src import models
    with Session(engine) as session, session.begin():
        admin = session.scalar(select(User).where(User.username == 'admin'))
        if admin is None:
            admin = User(username='admin', display_name='admin', password_hash=hash_password(password), feature_level='U3', knowledge_scope='all')
            session.add(admin)
            session.flush()
        admin.password_hash = hash_password(password)
        admin.status, admin.feature_level, admin.knowledge_scope = 'active', 'U3', 'all'
        admin_id = admin.id
        role = session.scalar(select(Role).where(Role.code == 'super_admin'))
        if role and not session.scalar(select(UserRole).where(UserRole.user_id == admin_id, UserRole.role_id == role.id)):
            session.add(UserRole(user_id=admin_id, role_id=role.id))
        for table in tables:
            cols = {col['name'] for col in inspector.get_columns(table)}
            if 'owner_id' in cols:
                session.execute(text(f'UPDATE `{table}` SET owner_id=:id'), {'id': admin_id})
            if 'created_by' in cols:
                session.execute(text(f'UPDATE `{table}` SET created_by=:id'), {'id': admin_id})
        session.execute(text('UPDATE users SET status=\'disabled\' WHERE id<>:id'), {'id': admin_id})
        session.execute(text('UPDATE refresh_tokens SET revoked_at=CURRENT_TIMESTAMP WHERE revoked_at IS NULL'))
    store.query('MATCH (n) WHERE n.owner_id IS NOT NULL SET n.owner_id=$id', id=admin_id)
    store.query('MATCH (n) WHERE n.owner IS NOT NULL SET n.owner=$name', name='admin')
    store.close()
    for source, target in copies:
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            shutil.copy2(source, target)
        assert digest(source) == digest(target)
    for folder in sources:
        destination = backup / 'planning-original-users' / folder.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        assert folder.resolve().parent == data.resolve() and destination.resolve().is_relative_to(backup)
        shutil.move(str(folder), str(destination))
    sys.path.insert(0, str(root))
    from backend.core.security import user_manager
    record = users.get('admin') or user_manager._build_user_record('admin', password, role='admin')
    salt, encoded = user_manager._hash_password(password)
    record.update(salt=salt, password_hash=encoded, role='admin', display_name='admin')
    user_manager.users = {'admin': record}
    user_manager._save()
    with user_manager._session_db() as db:
        db.execute('DELETE FROM sessions')
    (backup / 'complete.json').write_text(json.dumps({'admin_id': admin_id, 'completed': datetime.now().isoformat(), **summary}, indent=2), 'utf-8')
    print('Consolidation complete; all original files and ownership records backed up.')


if __name__ == '__main__':
    main()
