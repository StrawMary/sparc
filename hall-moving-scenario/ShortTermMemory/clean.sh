# clean every folder that needs to

# sterg clasificatorii
rm -r $(pwd)/ShortTermMemory/classifiers/*

# trebuie sa sterg toate fisierele pickle din fisierul de taskuri
rm -r $(pwd)/ShortTermMemory/tasks/*

# trebuie sa sterg orice fisier din short_time_memory files care nu trebuia sa fie acolo
find -path "*/temporary_aligned_faces_dir/*" -type d -not -name stefania -not -name alex -not -name andreea -not -name "." -exec rm -r {} \;
find -path "*/temporary_faces_dir/*" -type d -not -name stefania -not -name alex -not -name andreea -not -name "." -exec rm -r {} \;

# sa sterg orice din to_align_dir si aligned_dir
rm -r $(pwd)/ShortTermMemory/to_align_dir/*
rm -r $(pwd)/ShortTermMemory/aligned_dir/*
