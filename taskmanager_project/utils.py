def success(data=None):
    return_data = {
        'status': True,
        "message": "success"
    }
    if data:
        return_data['data'] = data
    return return_data


def fail(error):
    return_data = {
        'status': False,
        'message': 'fail',
        'error': error
    }
    return return_data
