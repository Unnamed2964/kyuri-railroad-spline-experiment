import type { ReactNode } from 'react';

type StepSectionProps = {
  accentClassName: string;
  index: string;
  title: string;
  children: ReactNode;
};

export function StepSection({ accentClassName, index, title, children }: StepSectionProps) {
  return (
    <section className="step-section" aria-labelledby={`step-${index}`}>
      <div className={`step-marker ${accentClassName}`}>{index}</div>
      <div>
        <h2 id={`step-${index}`}>{title}</h2>
        {children}
      </div>
    </section>
  );
}