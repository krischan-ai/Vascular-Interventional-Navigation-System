@echo off
REM Deploy backend runtime files to the remote Newton server and restart uvicorn.
REM Optional: set CATHSIM_DEPLOY_PASSWORD before running to avoid the prompt.
python tools\deploy_backend.py %*
