from flask import Flask, render_template, request, redirect, jsonify
import sqlite3
import requests

app = Flask(__name__)

DB_NAME = "smarthome.db"

# Home Assistant configuration
HOME_ASSISTANT_URL = "http://localhost:8123"
HOME_ASSISTANT_TOKEN = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiI2MTVhZDUwY2U4NTE0MDlmYTgyMGY4YWIzNDZlNWJhMCIsImlhdCI6MTc4MTkxNDAxOCwiZXhwIjoyMDk3Mjc0MDE4fQ.nR8wwZ8OoUV-Z_adyCOIZMDqDzQPr0r84FtCw5ptx4I"


def get_db():
    conn = sqlite3.connect(DB_NAME)
    conn.row_factory = sqlite3.Row
    return conn


def log_event(event_type, source, gesture_name, device_name, action, previous_state, new_state):
    db = get_db()
    db.execute(
        '''
        INSERT INTO EventLogs
        (event_type, source, gesture_name, device_name, action, previous_state, new_state)
        VALUES (?, ?, ?, ?, ?, ?, ?)
        ''',
        (event_type, source, gesture_name, device_name, action, previous_state, new_state)
    )
    db.commit()


def ha_api_call(endpoint, method="GET", data=None):
    """Make a request to the Home Assistant REST API."""
    url = f"{HOME_ASSISTANT_URL}{endpoint}"
    headers = {
        "Authorization": f"Bearer {HOME_ASSISTANT_TOKEN}",
        "Content-Type": "application/json"
    }

    try:
        if method == "GET":
            response = requests.get(url, headers=headers, timeout=5)
        elif method == "POST":
            response = requests.post(url, headers=headers, json=data, timeout=5)
        else:
            return {"success": False, "error": f"Método HTTP não suportado: {method}"}

        response.raise_for_status()
        return {"success": True, "data": response.json()}

    except requests.Timeout:
        return {"success": False, "error": "Timeout a comunicar com o Home Assistant"}
    except requests.HTTPError as e:
        return {"success": False, "error": f"Erro HTTP do Home Assistant: {e.response.status_code} - {e.response.text}"}
    except requests.ConnectionError:
        return {"success": False, "error": "Não foi possível conectar ao Home Assistant. Verifica se está em execução."}
    except Exception as e:
        return {"success": False, "error": str(e)}


def get_ha_entity_state(entity_id):
    """Get the current state of a Home Assistant entity."""
    result = ha_api_call(f"/api/states/{entity_id}", method="GET")
    if result["success"]:
        return result["data"].get("state")
    return None


def set_device_state(device_id, action, source="manual", gesture_name=None):
    db = get_db()

    device = db.execute(
        '''
        SELECT id, name, entity_id, current_state
        FROM Devices
        WHERE id = ?
        ''',
        (device_id,)
    ).fetchone()

    if not device:
        return {"sucesso": False, "erro": "Dispositivo não encontrado"}

    current_state = device["current_state"]
    entity_id = device["entity_id"]

    if action == "toggle":
        # Read actual state from Home Assistant
        ha_state = get_ha_entity_state(entity_id)
        if ha_state is None:
            # Fallback to local DB state
            ha_state = current_state
        new_state = "off" if ha_state == "on" else "on"
        ha_action = "turn_on" if new_state == "on" else "turn_off"
    elif action == "on":
        new_state = "on"
        ha_action = "turn_on"
    elif action == "off":
        new_state = "off"
        ha_action = "turn_off"
    else:
        return {"sucesso": False, "erro": "Ação inválida"}

    # Call Home Assistant service
    result = ha_api_call(
        f"/api/services/switch/{ha_action}",
        method="POST",
        data={"entity_id": entity_id}
    )

    if not result["success"]:
        return {"sucesso": False, "erro": result["error"]}

    # Update local database
    db.execute(
        'UPDATE Devices SET current_state = ? WHERE id = ?',
        (new_state, device_id)
    )
    db.commit()

    log_event(
        event_type="device_action",
        source=source,
        gesture_name=gesture_name,
        device_name=device["name"],
        action=action,
        previous_state=current_state,
        new_state=new_state
    )

    return {
        "sucesso": True,
        "device_id": device["id"],
        "device_name": device["name"],
        "novo_estado": new_state,
        "entity_id": entity_id
    }


@app.route('/')
def home():
    db = get_db()

    gestures = db.execute('SELECT * FROM Gestures ORDER BY name').fetchall()
    devices = db.execute('SELECT * FROM Devices ORDER BY name').fetchall()

    rules = db.execute('''
        SELECT Rules.id, Gestures.name AS g_name, Devices.name AS d_name, Rules.action
        FROM Rules
        JOIN Gestures ON Rules.gesture_id = Gestures.id
        JOIN Devices ON Rules.device_id = Devices.id
        ORDER BY Rules.id DESC
    ''').fetchall()

    logs = db.execute('''
        SELECT *
        FROM EventLogs
        ORDER BY id DESC
        LIMIT 10
    ''').fetchall()

    return render_template(
        'index.html',
        gestures=gestures,
        devices=devices,
        rules=rules,
        logs=logs
    )


@app.route('/add_gesture', methods=['POST'])
def add_gesture():
    gesture_name = request.form['gesture_name'].strip()

    if gesture_name:
        db = get_db()
        existing = db.execute(
            'SELECT id FROM Gestures WHERE lower(name) = lower(?)',
            (gesture_name,)
        ).fetchone()

        if not existing:
            db.execute('INSERT INTO Gestures (name) VALUES (?)', (gesture_name,))
            db.commit()

    return redirect('/')


@app.route('/delete_gesture/<int:gesture_id>', methods=['POST'])
def delete_gesture(gesture_id):
    db = get_db()
    db.execute('DELETE FROM Rules WHERE gesture_id = ?', (gesture_id,))
    db.execute('DELETE FROM Gestures WHERE id = ?', (gesture_id,))
    db.commit()
    return redirect('/')


@app.route('/add_device', methods=['POST'])
def add_device():
    device_name = request.form['device_name'].strip()
    entity_id = request.form['entity_id'].strip()

    if device_name and entity_id:
        db = get_db()
        existing = db.execute(
            'SELECT id FROM Devices WHERE lower(name) = lower(?) OR entity_id = ?',
            (device_name, entity_id)
        ).fetchone()

        if not existing:
            # Optionally verify the entity exists in Home Assistant
            ha_state = get_ha_entity_state(entity_id)
            initial_state = ha_state if ha_state else "off"

            db.execute(
                '''
                INSERT INTO Devices (name, entity_id, current_state)
                VALUES (?, ?, ?)
                ''',
                (device_name, entity_id, initial_state)
            )
            db.commit()

    return redirect('/')


@app.route('/delete_device/<int:device_id>', methods=['POST'])
def delete_device(device_id):
    db = get_db()
    db.execute('DELETE FROM Rules WHERE device_id = ?', (device_id,))
    db.execute('DELETE FROM Devices WHERE id = ?', (device_id,))
    db.commit()
    return redirect('/')


@app.route('/add_rule', methods=['POST'])
def add_rule():
    gesture_id = request.form['gesture_id']
    device_id = request.form['device_id']
    action = request.form['action']

    db = get_db()

    existing_rule = db.execute(
        '''
        SELECT id FROM Rules
        WHERE gesture_id = ? AND device_id = ?
        ''',
        (gesture_id, device_id)
    ).fetchone()

    if existing_rule:
        db.execute(
            '''
            UPDATE Rules
            SET action = ?
            WHERE id = ?
            ''',
            (action, existing_rule['id'])
        )
    else:
        db.execute(
            '''
            INSERT INTO Rules (gesture_id, device_id, action)
            VALUES (?, ?, ?)
            ''',
            (gesture_id, device_id, action)
        )

    db.commit()
    return redirect('/')


@app.route('/delete_rule/<int:rule_id>', methods=['POST'])
def delete_rule(rule_id):
    db = get_db()
    db.execute('DELETE FROM Rules WHERE id = ?', (rule_id,))
    db.commit()
    return redirect('/')


@app.route('/api/devices', methods=['GET'])
def api_devices():
    db = get_db()
    devices = db.execute(
        '''
        SELECT id, name, entity_id, current_state
        FROM Devices
        ORDER BY name
        '''
    ).fetchall()

    devices_list = []
    for d in devices:
        devices_list.append({
            "id": d["id"],
            "name": d["name"],
            "entity_id": d["entity_id"],
            "current_state": d["current_state"]
        })

    return jsonify(devices_list), 200


@app.route('/api/logs', methods=['GET'])
def api_logs():
    db = get_db()
    logs = db.execute(
        '''
        SELECT *
        FROM EventLogs
        ORDER BY id DESC
        LIMIT 10
        '''
    ).fetchall()

    logs_list = []
    for l in logs:
        logs_list.append({
            "id": l["id"],
            "event_type": l["event_type"],
            "source": l["source"],
            "gesture_name": l["gesture_name"],
            "device_name": l["device_name"],
            "action": l["action"],
            "previous_state": l["previous_state"],
            "new_state": l["new_state"],
            "created_at": l["created_at"]
        })

    return jsonify(logs_list), 200


@app.route('/api/device/<int:device_id>/action', methods=['POST'])
def api_device_action(device_id):
    data = request.get_json()

    if not data or 'action' not in data:
        return jsonify({"sucesso": False, "erro": "Nenhuma ação recebida"}), 400

    result = set_device_state(device_id, data['action'], source="manual")

    if result["sucesso"]:
        return jsonify(result), 200
    else:
        return jsonify(result), 400


@app.route('/api/trigger', methods=['POST'])
def trigger_from_bracelet():
    data = request.get_json()

    if not data or 'gesto' not in data:
        return jsonify({"erro": "Nenhum gesto recebido"}), 400

    detected_gesture = data['gesto']
    db = get_db()

    rules = db.execute(
        '''
        SELECT Devices.id AS device_id,
               Rules.action
        FROM Rules
        JOIN Gestures ON Rules.gesture_id = Gestures.id
        JOIN Devices ON Rules.device_id = Devices.id
        WHERE Gestures.name = ?
        ''',
        (detected_gesture,)
    ).fetchall()

    if not rules:
        log_event(
            event_type="gesture",
            source="bracelet",
            gesture_name=detected_gesture,
            device_name="none",
            action="no_rule",
            previous_state=None,
            new_state="ignored"
        )
        return jsonify({"info": "Gesto detetado mas sem regra associada"}), 404

    results = []

    for rule in rules:
        result = set_device_state(
            rule['device_id'],
            rule['action'],
            source="bracelet",
            gesture_name=detected_gesture
        )
        results.append(result)

    return jsonify({
        "sucesso": True,
        "gesto": detected_gesture,
        "resultados": results
    }), 200


@app.route('/api/ha/refresh', methods=['POST'])
def ha_refresh_states():
    """Refresh all device states from Home Assistant."""
    db = get_db()
    devices = db.execute('SELECT id, entity_id FROM Devices').fetchall()

    updated = []
    for device in devices:
        ha_state = get_ha_entity_state(device["entity_id"])
        if ha_state in ("on", "off"):
            db.execute(
                'UPDATE Devices SET current_state = ? WHERE id = ?',
                (ha_state, device["id"])
            )
            updated.append({"id": device["id"], "entity_id": device["entity_id"], "state": ha_state})

    db.commit()

    return jsonify({
        "sucesso": True,
        "dispositivos_atualizados": len(updated),
        "dispositivos": updated
    }), 200


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)