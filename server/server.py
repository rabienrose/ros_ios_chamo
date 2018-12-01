import os
from flask import Flask, flash, request, redirect, url_for
from werkzeug.utils import secure_filename

UPLOAD_FOLDER = '/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/iphone_upload'
ALLOWED_EXTENSIONS = set(['bag'])

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

@app.route('/upload_bag', methods=['GET', 'POST'])
def upload_bag():
    if request.method == 'POST':
        if 'file' not in request.files:
            flash('No file part')
            return redirect(request.url)
        file = request.files['file']
        if file.filename == '':
            flash('No selected file')
            return "no file"
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            print(app.config['UPLOAD_FOLDER'])
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
            return filename
    return ''

if __name__ == '__main__':
    app.config['SECRET_KEY'] = 'xxx'
    app.run('0.0.0.0', port=21070)