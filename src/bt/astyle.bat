REM ��������Ŀ¼�е�����C++�ļ���Astyle���д�����������
REM ����Astyle����λ�úͲ���
@echo off
set astyle="astyle.exe"
REM ѭ������Ŀ¼
for /r . %%a in (*.cpp;*.c) do %astyle% --style=allman --convert-tabs --indent=spaces=4 --attach-closing-while --indent-switches --indent-namespaces --indent-continuation=4 --indent-preproc-block --indent-preproc-define --indent-preproc-cond --indent-col1-comments --pad-oper --unpad-paren --delete-empty-lines --align-pointer=name --align-reference=name --break-elseifs --add-braces --break-blocks --indent-preproc-cond -s4 -n "%%a"
for /r . %%a in (*.hpp;*.h) do %astyle% --style=allman --convert-tabs --indent=spaces=4 --attach-closing-while --indent-switches --indent-namespaces --indent-continuation=4 --indent-preproc-block --indent-preproc-define --indent-preproc-cond --indent-col1-comments --pad-oper --unpad-paren --delete-empty-lines --align-pointer=name --align-reference=name --break-elseifs --add-braces --break-blocks --indent-preproc-cond -s4 -n "%%a"
REM ɾ�����е�astyle�����ļ�
for /r . %%a in (*.orig) do del "%%a"
pause