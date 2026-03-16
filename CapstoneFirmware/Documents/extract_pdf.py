import fitz
doc = fitz.open(r'c:\Users\mahjo\Downloads\Dorsheed STM Files-20260217T214918Z-1-001\Dorsheed STM Files\Documents\PI control parameters (1).pdf')
with open('tmp_pi_params.txt', 'w', encoding='utf-8') as f:
    f.write(' '.join([page.get_text() for page in doc]))
